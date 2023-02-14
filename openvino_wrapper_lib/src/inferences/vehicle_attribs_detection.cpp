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
 * @brief a realization file with declaration of VehicleAttribsDetection class and
 * VehicleAttribsDetectionResult class
 * @file vehicle_attribs_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/inferences/vehicle_attribs_detection.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// VehicleAttribsDetectionResult
openvino_wrapper_lib::VehicleAttribsDetectionResult::VehicleAttribsDetectionResult(
  const cv::Rect & location)
: Result(location) {}

// VehicleAttribsDetection
openvino_wrapper_lib::VehicleAttribsDetection::VehicleAttribsDetection()
: openvino_wrapper_lib::BaseInference() {}

openvino_wrapper_lib::VehicleAttribsDetection::~VehicleAttribsDetection() = default;
void openvino_wrapper_lib::VehicleAttribsDetection::loadNetwork(
  const std::shared_ptr<Models::VehicleAttribsDetectionModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool openvino_wrapper_lib::VehicleAttribsDetection::enqueue(
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

bool openvino_wrapper_lib::VehicleAttribsDetection::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::VehicleAttribsDetection::fetchResults()
{
  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;

  ov::InferRequest infer_request = getEngine()->getRequest();
  std::string color_name = valid_model_->getOutputName("color_output_");
  std::string type_name = valid_model_->getOutputName("type_output_");
  const float * color_values = infer_request.get_tensor(color_name).data<float>();
  const float * type_values = infer_request.get_tensor(type_name).data<float>();

  for (int i = 0; i < getResultsLength(); i++) {
    auto color_id = std::max_element(color_values, color_values + 7) - color_values;
    auto type_id = std::max_element(type_values, type_values + 4) - type_values;
    color_values += 7;
    type_values += 4;
    results_[i].color_ = colors_[color_id];
    results_[i].type_ = types_[type_id];
    found_result = true;
  }
  if (!found_result) {results_.clear();}
  return true;
}

int openvino_wrapper_lib::VehicleAttribsDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::VehicleAttribsDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::VehicleAttribsDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::VehicleAttribsDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::VehicleAttribsDetection::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << "Vehicle attributes detection does not support filtering now! " <<
      "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
