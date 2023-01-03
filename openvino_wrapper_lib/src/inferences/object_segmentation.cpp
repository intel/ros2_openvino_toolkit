// Copyright (c) 2018-2022 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief a header file with declaration of ObjectSegmentation class and
 * ObjectSegmentationResult class
 * @file object_segmentation.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <random>

#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/inferences/object_segmentation.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// ObjectSegmentationResult
openvino_wrapper_lib::ObjectSegmentationResult::ObjectSegmentationResult(const cv::Rect &location)
    : Result(location)
{
}

// ObjectSegmentation
openvino_wrapper_lib::ObjectSegmentation::ObjectSegmentation(double show_output_thresh)
    : show_output_thresh_(show_output_thresh), openvino_wrapper_lib::BaseInference()
{
}

openvino_wrapper_lib::ObjectSegmentation::~ObjectSegmentation() = default;

void openvino_wrapper_lib::ObjectSegmentation::loadNetwork(
    const std::shared_ptr<Models::ObjectSegmentationModel> network)
{
  slog::info << "Loading Network: " << network->getModelCategory() << slog::endl;
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

/**
 * Deprecated!
 * This function only support OpenVINO version <=2018R5
 */
bool openvino_wrapper_lib::ObjectSegmentation::enqueue_for_one_input(
    const cv::Mat &frame,
    const cv::Rect &input_frame_loc)
{
  if (width_ == 0 && height_ == 0)
  {
    width_ = frame.cols;
    height_ = frame.rows;
  }
  if (!openvino_wrapper_lib::BaseInference::enqueue<u_int8_t>(frame, input_frame_loc, 1, 0,
    valid_model_->getInputName()))
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}

bool openvino_wrapper_lib::ObjectSegmentation::enqueue(
    const cv::Mat &frame,
    const cv::Rect &input_frame_loc)
{
  if (width_ == 0 && height_ == 0)
  {
    width_ = frame.cols;
    height_ = frame.rows;
  }

  if (valid_model_ == nullptr || getEngine() == nullptr)
  {
    throw std::logic_error("Model or Engine is not set correctly!");
    return false;
  }

  if (enqueued_frames_ >= valid_model_->getMaxBatchSize())
  {
    slog::warn << "Number of " << getName() << "input more than maximum(" <<
      max_batch_size_ << ") processed by inference" << slog::endl;
    return false;
  }

  if (!valid_model_->enqueue(getEngine(), frame, input_frame_loc))
  {
    return false;
  }

  enqueued_frames_ += 1;
  return true;
}

bool openvino_wrapper_lib::ObjectSegmentation::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::ObjectSegmentation::fetchResults()
{
  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch)
  {
    return false;
  }
  bool found_result = false;
  results_.clear();
  ov::InferRequest infer_request = getEngine()->getRequest();
  slog::debug << "Analyzing Detection results..." << slog::endl;
  std::string detection_output = valid_model_->getOutputName("detection");

  ov::Tensor output_tensor = infer_request.get_tensor(detection_output);
  const auto out_data = output_tensor.data<float>();
  ov::Shape out_shape = output_tensor.get_shape();
  size_t output_w, output_h, output_des, output_extra = 0;
  if (out_shape.size() == 3) {
    output_w = out_shape[2];
    output_h = out_shape[1];
    output_des = out_shape[0];
  } else if (out_shape.size() == 4) {
    output_w = out_shape[3];
    output_h = out_shape[2];
    output_des = out_shape[1];
    output_extra = out_shape[0];
  } else {
    slog::warn << "unexpected output shape: " <<out_shape << slog::endl;
    return false;
  }

  slog::debug << "output w " << output_w<< slog::endl;
  slog::debug << "output h " << output_h << slog::endl;
  slog::debug << "output description " << output_des << slog::endl;
  slog::debug << "output extra " << output_extra << slog::endl;

  const float *detections = output_tensor.data<float>();
  std::vector<std::string> &labels = valid_model_->getLabels();
  slog::debug << "label size " <<labels.size() << slog::endl;

  cv::Mat inImg, resImg, maskImg(output_h, output_w, CV_8UC3);
  cv::Mat colored_mask(output_h, output_w, CV_8UC3);
  cv::Rect roi = cv::Rect(0, 0, output_w, output_h);

  for (int rowId = 0; rowId < output_h; ++rowId)
  {
    for (int colId = 0; colId < output_w; ++colId)
    {
      std::size_t classId = 0;
      float maxProb = -1.0f;
      if (output_des < 2) {  // assume the output is already ArgMax'ed
        classId = static_cast<std::size_t>(detections[rowId * output_w + colId]);
        for (int ch = 0; ch < colored_mask.channels();++ch){
          colored_mask.at<cv::Vec3b>(rowId, colId)[ch] = colors_[classId][ch];
        }
      } else {
        for (int chId = 0; chId < output_des; ++chId)
        {
          float prob = detections[chId * output_h * output_w + rowId * output_w+ colId];
          if (prob > maxProb)
          {
            classId = chId;
            maxProb = prob;
          }
        }
        while (classId >= colors_.size())
        {
          static std::mt19937 rng(classId);
          std::uniform_int_distribution<int> distr(0, 255);
          cv::Vec3b color(distr(rng), distr(rng), distr(rng));
          colors_.push_back(color);
        }
        if(maxProb > 0.5){
        for (int ch = 0; ch < colored_mask.channels();++ch){
          colored_mask.at<cv::Vec3b>(rowId, colId)[ch] = colors_[classId][ch];
        }
       }
      }
    }
  }
  const float alpha = 0.7f;
  Result result(roi);
  result.mask_ = colored_mask;
  found_result = true;
  results_.emplace_back(result);
  return true;
}

int openvino_wrapper_lib::ObjectSegmentation::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::ObjectSegmentation::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::ObjectSegmentation::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::ObjectSegmentation::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput> &output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::ObjectSegmentation::getFilteredROIs(
    const std::string filter_conditions) const
{
  if (!filter_conditions.empty())
  {
    slog::err << "Object segmentation does not support filtering now! "
              << "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  return filtered_rois;
}
