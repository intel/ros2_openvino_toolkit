// Copyright (c) 2026 Intel Corporation
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
 * @brief Implementation of the ModelInference class and its instantiations
 * @file model_inference.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/inferences/model_inference.hpp"
#include "openvino_wrapper_lib/models/depth_estimation_yolo26_model.hpp"
#include "openvino_wrapper_lib/models/image_classification_yolo26_model.hpp"
#include "openvino_wrapper_lib/models/keypoint_detection_yolo26_model.hpp"
#include "openvino_wrapper_lib/models/oriented_object_detection_yolo26_model.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

namespace openvino_wrapper_lib
{
template <typename ModelT, typename ResultT>
ModelInference<ModelT, ResultT>::ModelInference(double show_output_thresh) : show_output_thresh_(show_output_thresh)
{
}

template <typename ModelT, typename ResultT>
void ModelInference<ModelT, ResultT>::loadNetwork(std::shared_ptr<ModelT> network)
{
  slog::info << "Loading Network: " << network->getModelCategory() << slog::endl;
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

template <typename ModelT, typename ResultT>
bool ModelInference<ModelT, ResultT>::enqueue(const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  if (valid_model_ == nullptr || getEngine() == nullptr) {
    throw std::logic_error("Model or Engine is not set correctly!");
  }

  if (enqueued_frames_ >= valid_model_->getMaxBatchSize()) {
    slog::warn << "Number of " << getName() << "input more than maximum(" << max_batch_size_
               << ") processed by inference" << slog::endl;
    return false;
  }

  if (!valid_model_->enqueue(getEngine(), frame, input_frame_loc)) {
    return false;
  }

  enqueued_frames_ += 1;
  return true;
}

template <typename ModelT, typename ResultT>
bool ModelInference<ModelT, ResultT>::fetchResults()
{
  if (!BaseInference::fetchResults()) {
    return false;
  }
  results_.clear();

  return valid_model_ != nullptr && valid_model_->fetchResults(getEngine(), results_, show_output_thresh_);
}

template <typename ModelT, typename ResultT>
int ModelInference<ModelT, ResultT>::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

template <typename ModelT, typename ResultT>
const Result* ModelInference<ModelT, ResultT>::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

template <typename ModelT, typename ResultT>
const std::string ModelInference<ModelT, ResultT>::getName() const
{
  return valid_model_->getModelCategory();
}

template <typename ModelT, typename ResultT>
void ModelInference<ModelT, ResultT>::observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

template <typename ModelT, typename ResultT>
const std::vector<cv::Rect> ModelInference<ModelT, ResultT>::getFilteredROIs(const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << getName() << " does not support filtering now! "
              << "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (const auto& res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}

template class ModelInference<Models::KeypointDetectionYolo26Model, KeypointDetectionResult>;
template class ModelInference<Models::OrientedObjectDetectionYolo26Model, OrientedObjectDetectionResult>;
template class ModelInference<Models::ImageClassificationYolo26Model, ImageClassificationResult>;
template class ModelInference<Models::DepthEstimationYolo26Model, DepthEstimationResult>;
}  // namespace openvino_wrapper_lib
