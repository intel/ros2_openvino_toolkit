// Copyright (c) 2018 Intel Corporation
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
#include "dynamic_vino_lib/inferences/person_attribs_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// PersonAttribsDetectionResult
dynamic_vino_lib::PersonAttribsDetectionResult::PersonAttribsDetectionResult(
  const cv::Rect & location)
: Result(location) {}

// PersonAttribsDetection
dynamic_vino_lib::PersonAttribsDetection::PersonAttribsDetection(double attribs_confidence)
: attribs_confidence_(attribs_confidence), dynamic_vino_lib::BaseInference() {}

dynamic_vino_lib::PersonAttribsDetection::~PersonAttribsDetection() = default;
void dynamic_vino_lib::PersonAttribsDetection::loadNetwork(
  const std::shared_ptr<Models::PersonAttribsDetectionModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::PersonAttribsDetection::enqueue(
  const cv::Mat & frame, const cv::Rect & input_frame_loc)
{
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  if (!dynamic_vino_lib::BaseInference::enqueue<u_int8_t>(
      frame, input_frame_loc, 1, 0, valid_model_->getInputName()))
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::PersonAttribsDetection::submitRequest()
{
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::PersonAttribsDetection::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float * output_values = request->GetBlob(output)->buffer().as<float *>();
  int net_attrib_length = net_attributes_.size();
  for (int i = 0; i < getResultsLength(); i++) {
    results_[i].male_probability_ = output_values[i * net_attrib_length];
    std::string attrib = "";
    for (int j = 1; j < net_attrib_length; j++) {
      attrib += (output_values[i * net_attrib_length + j] > attribs_confidence_) ?
        net_attributes_[j] + ", " : "";
    }
    results_[i].attributes_ = attrib;
    found_result = true;
  }
  if (!found_result) {results_.clear();}
  return true;
}

int dynamic_vino_lib::PersonAttribsDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result *
dynamic_vino_lib::PersonAttribsDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::PersonAttribsDetection::getName() const
{
  return valid_model_->getModelName();
}

void dynamic_vino_lib::PersonAttribsDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> dynamic_vino_lib::PersonAttribsDetection::getFilteredROIs(
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
