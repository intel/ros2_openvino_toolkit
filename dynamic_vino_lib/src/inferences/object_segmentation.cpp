/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @brief a header file with declaration of ObjectSegmentation class and
 * ObjectSegmentationResult class
 * @file object_segmentation.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "dynamic_vino_lib/inferences/object_segmentation.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// ObjectSegmentationResult
dynamic_vino_lib::ObjectSegmentationResult::ObjectSegmentationResult(
    const cv::Rect& location)
    : Result(location){}

// ObjectSegmentation
dynamic_vino_lib::ObjectSegmentation::ObjectSegmentation(double show_output_thresh)
    : show_output_thresh_(show_output_thresh),
      dynamic_vino_lib::BaseInference(){}

dynamic_vino_lib::ObjectSegmentation::~ObjectSegmentation() = default;

void dynamic_vino_lib::ObjectSegmentation::loadNetwork(
    const std::shared_ptr<Models::ObjectSegmentationModel> network) {
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::ObjectSegmentation::enqueue(const cv::Mat& frame,
    const cv::Rect& input_frame_loc) {
  if (width_ == 0 && height_ == 0) {
    width_ = frame.cols;
    height_ = frame.rows;
  }
  if (!dynamic_vino_lib::BaseInference::enqueue<u_int8_t>(
          frame, input_frame_loc, 1, 0, valid_model_->getInputName())) {
    return false;
  }
  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::ObjectSegmentation::submitRequest() {
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::ObjectSegmentation::fetchResults() {
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) return false;
  bool found_result = false;
  results_.clear();
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string detection_output = valid_model_->getDetectionOutputName();
  std::string mask_output = valid_model_->getMaskOutputName();
  const auto do_blob = request->GetBlob(detection_output.c_str());
  const auto do_data = do_blob->buffer().as<float*>();
  const auto masks_blob = request->GetBlob(mask_output.c_str());
  const auto masks_data = masks_blob->buffer().as<float*>();
  // amount of elements in each detected box description (batch, label, prob, x1, y1, x2, y2)
  size_t box_num = masks_blob->dims().at(3);
  size_t label_num = masks_blob->dims().at(2);
  size_t box_description_size = do_blob->dims().at(0);
  size_t H = masks_blob->dims().at(1);
  size_t W = masks_blob->dims().at(0);
  size_t box_stride = W * H * label_num;
  for (size_t box = 0; box < box_num; ++box) {
      float* box_info = do_data + box * box_description_size;
      float batch = box_info[0];
      if (batch < 0) break;
      float prob = box_info[2];
      if (prob > show_output_thresh_) {
          float x1 = std::min(std::max(0.0f, box_info[3] * width_), static_cast<float>(width_));
          float y1 = std::min(std::max(0.0f, box_info[4] * height_), static_cast<float>(height_));
          float x2 = std::min(std::max(0.0f, box_info[5] * width_), static_cast<float>(width_));
          float y2 = std::min(std::max(0.0f, box_info[6] * height_), static_cast<float>(height_));
          int box_width = std::min(static_cast<int>(std::max(0.0f, x2 - x1)), width_);
          int box_height = std::min(static_cast<int>(std::max(0.0f, y2 - y1)), height_);
          int class_id = static_cast<int>(box_info[1] + 1e-6f);
          float* mask_arr = masks_data + box_stride * box + H * W * (class_id - 1);
          cv::Mat mask_mat(H, W, CV_32FC1, mask_arr);
          cv::Rect roi = cv::Rect(
            static_cast<int>(x1), static_cast<int>(y1), box_width, box_height);
          cv::Mat resized_mask_mat(box_height, box_width, CV_32FC1);
          cv::resize(mask_mat, resized_mask_mat, cv::Size(box_width, box_height));
          Result result(roi);
          result.confidence_ = prob;
          std::vector<std::string>& labels = valid_model_->getLabels();
          result.label_ = class_id < labels.size()
                        ? labels[class_id]
                        : std::string("label #") + std::to_string(class_id);
          result.mask_ = resized_mask_mat;
          found_result = true;
          results_.emplace_back(result);
      }
  }
  if (!found_result) results_.clear();
  return true;
}

const int dynamic_vino_lib::ObjectSegmentation::getResultsLength() const {
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result*
dynamic_vino_lib::ObjectSegmentation::getLocationResult(int idx) const {
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::ObjectSegmentation::getName() const {
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::ObjectSegmentation::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output) {
  if (output != nullptr) {
    output->accept(results_);
  }
}
