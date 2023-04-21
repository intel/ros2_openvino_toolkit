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
 * @brief a header file with declaration of FaceReidentification class and
 * FaceReidentificationResult class
 * @file face_reidentification.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/inferences/face_reidentification.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// FaceReidentificationResult
openvino_wrapper_lib::FaceReidentificationResult::FaceReidentificationResult(
  const cv::Rect & location)
: Result(location) {}

// FaceReidentification
openvino_wrapper_lib::FaceReidentification::FaceReidentification(double match_thresh)
: openvino_wrapper_lib::BaseInference()
{
  face_tracker_ = std::make_shared<openvino_wrapper_lib::Tracker>(1000, match_thresh, 0.3);
}

openvino_wrapper_lib::FaceReidentification::~FaceReidentification() = default;
void openvino_wrapper_lib::FaceReidentification::loadNetwork(
  const std::shared_ptr<Models::FaceReidentificationModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool openvino_wrapper_lib::FaceReidentification::enqueue(
  const cv::Mat & frame, const cv::Rect & input_frame_loc)
{
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  if (!openvino_wrapper_lib::BaseInference::enqueue<u_int8_t>(
      frame, input_frame_loc, 1, 0, valid_model_->getInputName()))
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool openvino_wrapper_lib::FaceReidentification::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::FaceReidentification::fetchResults()
{
  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float * output_values = request->GetBlob(output)->buffer().as<float *>();
  int result_length = request->GetBlob(output)->getTensorDesc().getDims()[1];
  for (int i = 0; i < getResultsLength(); i++) {
    std::vector<float> new_face = std::vector<float>(
      output_values + result_length * i, output_values + result_length * (i + 1));
    std::string face_id = "No." + std::to_string(face_tracker_->processNewTrack(new_face));
    results_[i].face_id_ = face_id;
    found_result = true;
  }
  if (!found_result) {results_.clear();}
  return true;
}

int openvino_wrapper_lib::FaceReidentification::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::FaceReidentification::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::FaceReidentification::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::FaceReidentification::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::FaceReidentification::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << "Face reidentification does not support filtering now! " <<
      "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
