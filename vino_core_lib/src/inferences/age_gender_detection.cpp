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
 * @brief a header file with declaration of AgeGenderResult class
 * @file age_gender_detection.cpp
 */

#include <string>
#include <memory>
#include <vector>
#include "vino_core_lib/inferences/age_gender_detection.hpp"
#include "vino_core_lib/outputs/base_output.hpp"

// AgeGenderResult
vino_core_lib::AgeGenderResult::AgeGenderResult(const cv::Rect & location)
: Result(location)
{
}

// AgeGender Detection
vino_core_lib::AgeGenderDetection::AgeGenderDetection()
: vino_core_lib::BaseInference()
{
}

vino_core_lib::AgeGenderDetection::~AgeGenderDetection() = default;

void vino_core_lib::AgeGenderDetection::loadNetwork(
  std::shared_ptr<Models::AgeGenderDetectionModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool vino_core_lib::AgeGenderDetection::enqueue(
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  bool succeed = vino_core_lib::BaseInference::enqueue<float>(
    frame, input_frame_loc, 1, getResultsLength(), valid_model_->getInputName());
  if (!succeed) {
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool vino_core_lib::AgeGenderDetection::submitRequest()
{
  return vino_core_lib::BaseInference::submitRequest();
}

bool vino_core_lib::AgeGenderDetection::fetchResults()
{
  bool can_fetch = vino_core_lib::BaseInference::fetchResults();
  if (!can_fetch) {
    return false;
  }
  auto request = getEngine()->getRequest();
  InferenceEngine::Blob::Ptr genderBlob = request->GetBlob(valid_model_->getOutputGenderName());
  InferenceEngine::Blob::Ptr ageBlob = request->GetBlob(valid_model_->getOutputAgeName());

  for (int i = 0; i < results_.size(); ++i) {
    results_[i].age_ = ageBlob->buffer().as<float *>()[i] * 100;
    results_[i].male_prob_ = genderBlob->buffer().as<float *>()[i * 2 + 1];
  }
  return true;
}

int vino_core_lib::AgeGenderDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const vino_core_lib::Result *
vino_core_lib::AgeGenderDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string vino_core_lib::AgeGenderDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void vino_core_lib::AgeGenderDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> vino_core_lib::AgeGenderDetection::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << "Age gender detection does not support filtering now! " <<
      "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
