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
 * @brief a realization file with declaration of LicensePlateDetection class and
 * LicensePlateDetectionResult class
 * @file license_plate_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "dynamic_vino_lib/inferences/license_plate_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// LicensePlateDetectionResult
dynamic_vino_lib::LicensePlateDetectionResult::LicensePlateDetectionResult(
  const cv::Rect & location)
: Result(location) {}

// LicensePlateDetection
dynamic_vino_lib::LicensePlateDetection::LicensePlateDetection()
: dynamic_vino_lib::BaseInference() {}

dynamic_vino_lib::LicensePlateDetection::~LicensePlateDetection() = default;
void dynamic_vino_lib::LicensePlateDetection::loadNetwork(
  const std::shared_ptr<Models::LicensePlateDetectionModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

void dynamic_vino_lib::LicensePlateDetection::fillSeqBlob()
{
  InferenceEngine::Blob::Ptr seq_blob = getEngine()->getRequest()->GetBlob(
    valid_model_->getSeqInputName());
  int max_sequence_size = seq_blob->getTensorDesc().getDims()[0];
  // second input is sequence, which is some relic from the training
  // it should have the leading 0.0f and rest 1.0f
  float * blob_data = seq_blob->buffer().as<float *>();
  blob_data[0] = 0.0f;
  std::fill(blob_data + 1, blob_data + max_sequence_size, 1.0f);
}

bool dynamic_vino_lib::LicensePlateDetection::enqueue(
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
  fillSeqBlob();
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::LicensePlateDetection::submitRequest()
{
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::LicensePlateDetection::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float * output_values = request->GetBlob(output)->buffer().as<float *>();
  for (int i = 0; i < getResultsLength(); i++) {
    std::string license = "";
    int max_size = valid_model_->getMaxSequenceSize();
    for (int j = 0; j < max_size; j++) {
      if (output_values[i * max_size + j] == -1) {
        break;
      }
      license += licenses_[output_values[i * max_size + j]];
    }
    results_[i].license_ = license;
    found_result = true;
  }
  if (!found_result) {results_.clear();}
  return true;
}

const int dynamic_vino_lib::LicensePlateDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result *
dynamic_vino_lib::LicensePlateDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::LicensePlateDetection::getName() const
{
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::LicensePlateDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> dynamic_vino_lib::LicensePlateDetection::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << "License plate detection does not support filtering now! " <<
      "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
