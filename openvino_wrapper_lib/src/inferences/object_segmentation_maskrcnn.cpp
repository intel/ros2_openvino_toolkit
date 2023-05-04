// Copyright (c) 2022 Intel Corporation
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
#include "openvino_wrapper_lib/inferences/object_segmentation_maskrcnn.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// ObjectSegmentationResult
openvino_wrapper_lib::ObjectSegmentationMaskrcnnResult::ObjectSegmentationMaskrcnnResult(const cv::Rect &location)
    : Result(location)
{
}

// ObjectSegmentation
openvino_wrapper_lib::ObjectSegmentationMaskrcnn::ObjectSegmentationMaskrcnn(double show_output_thresh)
    : show_output_thresh_(show_output_thresh), openvino_wrapper_lib::BaseInference()
{
}

openvino_wrapper_lib::ObjectSegmentationMaskrcnn::~ObjectSegmentationMaskrcnn() = default;

void openvino_wrapper_lib::ObjectSegmentationMaskrcnn::loadNetwork(
    const std::shared_ptr<Models::ObjectSegmentationMaskrcnnModel> network)
{
  slog::info << "Loading Network: " << network->getModelCategory() << slog::endl;
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

/**
 * Deprecated!
 * This function only support OpenVINO version <=2018R5
 */
bool openvino_wrapper_lib::ObjectSegmentationMaskrcnn::enqueue_for_one_input(
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

bool openvino_wrapper_lib::ObjectSegmentationMaskrcnn::enqueue(
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

bool openvino_wrapper_lib::ObjectSegmentationMaskrcnn::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::ObjectSegmentationMaskrcnn::fetchResults()
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
  std::string mask_output = valid_model_->getOutputName("masks");
  slog::debug << "Detection_output=" << detection_output << ", Mask_output=" << mask_output << slog::endl;
  
  //get detection data
  ov::Tensor do_tensor = infer_request.get_tensor(detection_output.c_str());
  const auto do_data = do_tensor.data<float>();
  ov::Shape do_shape = do_tensor.get_shape();
  slog::debug << "Detection Blob getDims = " <<do_shape.size() << "[Should be 2]" << slog::endl;
  // get mask data
  ov::Tensor mask_tensor = infer_request.get_tensor(mask_output.c_str());
  const auto mask_data = mask_tensor.data<float>();
  ov::Shape mask_shape = mask_tensor.get_shape();

  // determine models
  size_t box_description_size = do_shape.back();
  OPENVINO_ASSERT(mask_shape.size() == 4);
  size_t box_num = mask_shape[0];
  size_t C = mask_shape[1];
  size_t H = mask_shape[2];
  size_t W = mask_shape[3];
  size_t box_stride = W * H * C;
  slog::debug << "box_description is:" << box_description_size << slog::endl;
  slog::debug << "box_num is:" << box_num<< slog::endl;
  slog::debug << "C is:" << C << slog::endl;
  slog::debug << "H is:" << H << slog::endl;
  slog::debug << "W is:" << W << slog::endl;

  for (size_t box = 0; box < box_num; ++box) {
    // box description: batch, label, prob, x1, y1, x2, y2
    float * box_info = do_data + box * box_description_size;
    auto batch = static_cast<int>(box_info[0]);
    slog::debug << "batch =" << batch << slog::endl;
    if (batch < 0) {
      slog::warn << "Batch size should be greater than 0. [batch=" << batch <<"]." << slog::endl;
      break;
    }
    float prob = box_info[2];
    if (prob > show_output_thresh_) {
      float x1 = std::min(std::max(0.0f, box_info[3] * width_), static_cast<float>(width_));
      float y1 = std::min(std::max(0.0f, box_info[4] * height_), static_cast<float>(height_));
      float x2 = std::min(std::max(0.0f, box_info[5] * width_), static_cast<float>(width_));
      float y2 = std::min(std::max(0.0f, box_info[6] * height_), static_cast<float>(height_));
      int box_width = static_cast<int>(x2 - x1);
      int box_height = static_cast<int>(y2 - y1);
      slog::debug << "Box[" << box_width << "x" << box_height << "]" << slog::endl;
      if (box_width <= 0 || box_height <=0) break;
      int class_id = static_cast<int>(box_info[1] + 1e-6f);
      float * mask_arr = mask_data + box_stride * box + H * W * (class_id - 1);
      slog::info << "Detected class " << class_id << " with probability " << prob << " from batch " << batch
                          << ": [" << x1 << ", " << y1 << "], [" << x2 << ", " << y2 << "]" << slog::endl;
      cv::Mat mask_mat(H, W, CV_32FC1, mask_arr);
      cv::Rect roi = cv::Rect(static_cast<int>(x1), static_cast<int>(y1), box_width, box_height);
      cv::Mat resized_mask_mat(box_height, box_width, CV_32FC1);
      cv::resize(mask_mat, resized_mask_mat, cv::Size(box_width, box_height));
      Result result(roi);
      result.confidence_ = prob;
      std::vector<std::string> & labels = valid_model_->getLabels();
      result.label_ = class_id < labels.size() ? labels[class_id] :
        std::string("label #") + std::to_string(class_id);
      result.mask_ = resized_mask_mat;
      found_result = true;
      slog::debug << "adding one segmentation Box ..." << slog::endl;
      results_.emplace_back(result);
    }
  }
  if (!found_result) {
    slog::debug << "No Segmentation Result Found!" << slog::endl;
    results_.clear();
  }
  return true;
}

int openvino_wrapper_lib::ObjectSegmentationMaskrcnn::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::ObjectSegmentationMaskrcnn::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::ObjectSegmentationMaskrcnn::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::ObjectSegmentationMaskrcnn::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput> &output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::ObjectSegmentationMaskrcnn::getFilteredROIs(
    const std::string filter_conditions) const
{
  if (!filter_conditions.empty())
  {
    slog::err << "Object segmentation does not support filtering now! "
              << "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_)
  {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
