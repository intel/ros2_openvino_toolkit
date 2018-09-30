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
 * @brief a header file with declaration of AgeGenderResult class
 * @file age_gender_detection.cpp
 */

#include <string>
#include <memory>

#include "dynamic_vino_lib/inferences/age_gender_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"

// AgeGenderResult
dynamic_vino_lib::AgeGenderResult::AgeGenderResult(const cv::Rect& location)
    : Result(location){}

// AgeGender Detection
dynamic_vino_lib::AgeGenderDetection::AgeGenderDetection()
    : dynamic_vino_lib::BaseInference(){}

dynamic_vino_lib::AgeGenderDetection::~AgeGenderDetection() = default;

void dynamic_vino_lib::AgeGenderDetection::loadNetwork(
    std::shared_ptr<Models::AgeGenderDetectionModel> network) {
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::AgeGenderDetection::enqueue(
    const cv::Mat& frame, const cv::Rect& input_frame_loc) {
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  bool succeed = dynamic_vino_lib::BaseInference::enqueue<float>(
      frame, input_frame_loc, 1, getResultsLength(),
      valid_model_->getInputName());
  if (!succeed) return false;
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::AgeGenderDetection::submitRequest() {
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::AgeGenderDetection::fetchResults() {
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) return false;
  auto request = getEngine()->getRequest();
  InferenceEngine::Blob::Ptr genderBlob =
      request->GetBlob(valid_model_->getOutputGenderName());
  InferenceEngine::Blob::Ptr ageBlob =
      request->GetBlob(valid_model_->getOutputAgeName());

  for (int i = 0; i < results_.size(); ++i) {
    results_[i].age_ = ageBlob->buffer().as<float*>()[i] * 100;
    results_[i].male_prob_ = genderBlob->buffer().as<float*>()[i * 2 + 1];
  }
  return true;
}

const int dynamic_vino_lib::AgeGenderDetection::getResultsLength() const {
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result*
dynamic_vino_lib::AgeGenderDetection::getLocationResult(int idx) const {
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::AgeGenderDetection::getName() const {
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::AgeGenderDetection::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output) {
  if (output != nullptr) {
    output->accept(results_);
  }
}
