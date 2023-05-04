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
 * @brief a header file with declaration of PersonAttribsDetection class and
 * PersonAttribsDetectionResult class
 * @file person_attribs_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/inferences/person_attribs_detection.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// PersonAttribsDetectionResult
openvino_wrapper_lib::PersonAttribsDetectionResult::PersonAttribsDetectionResult(
  const cv::Rect & location)
: Result(location) {}

// PersonAttribsDetection
openvino_wrapper_lib::PersonAttribsDetection::PersonAttribsDetection(double attribs_confidence)
: attribs_confidence_(attribs_confidence), openvino_wrapper_lib::BaseInference() {}

openvino_wrapper_lib::PersonAttribsDetection::~PersonAttribsDetection() = default;
void openvino_wrapper_lib::PersonAttribsDetection::loadNetwork(
  const std::shared_ptr<Models::PersonAttribsDetectionModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool openvino_wrapper_lib::PersonAttribsDetection::enqueue(
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

bool openvino_wrapper_lib::PersonAttribsDetection::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::PersonAttribsDetection::fetchResults()
{
  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;
  ov::InferRequest request = getEngine()->getRequest();
  slog::debug << "Analyzing Attributes Detection results..." << slog::endl;
  std::string attribute_output = valid_model_->getOutputName("attributes_output_");
  std::string top_output = valid_model_->getOutputName("top_output_");
  std::string bottom_output = valid_model_->getOutputName("bottom_output_");

  ov::Tensor attrib_tensor = request.get_tensor(attribute_output);
  ov::Tensor top_tensor = request.get_tensor(top_output);
  ov::Tensor bottom_tensor = request.get_tensor(bottom_output);

  auto attri_values = attrib_tensor.data<float>();
  auto top_values = top_tensor.data<float>();
  auto bottom_values = bottom_tensor.data<float>();

  int net_attrib_length = net_attributes_.size();
  for (int i = 0; i < getResultsLength(); i++) {
    results_[i].male_probability_ = attri_values[i * net_attrib_length];
    results_[i].top_point_.x = top_values[i];
    results_[i].top_point_.y = top_values[i+1];
    results_[i].bottom_point_.x = bottom_values[i];
    results_[i].bottom_point_.y = bottom_values[i+1];
    std::string attrib = "";
    for (int j = 1; j < net_attrib_length; j++) {
      attrib += (attri_values[i * net_attrib_length + j] > attribs_confidence_) ?
        net_attributes_[j] + ", " : "";
    }
    results_[i].attributes_ = attrib;    

    found_result = true;
  }  
  if (!found_result) {results_.clear();}
  return true;
}

int openvino_wrapper_lib::PersonAttribsDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::PersonAttribsDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::PersonAttribsDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::PersonAttribsDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::PersonAttribsDetection::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << "Person attributes detection does not support filtering now! " <<
      "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
