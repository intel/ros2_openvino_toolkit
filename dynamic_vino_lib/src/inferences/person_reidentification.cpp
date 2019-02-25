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
 * @brief a header file with declaration of PersonReidentification class and
 * PersonReidentificationResult class
 * @file person_reidentification.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "dynamic_vino_lib/inferences/person_reidentification.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// PersonReidentificationResult
dynamic_vino_lib::PersonReidentificationResult::PersonReidentificationResult(
    const cv::Rect& location)
    : Result(location){}

// PersonReidentification
dynamic_vino_lib::PersonReidentification::PersonReidentification(double match_thresh)
    : match_thresh_(match_thresh),dynamic_vino_lib::BaseInference(){}

dynamic_vino_lib::PersonReidentification::~PersonReidentification() = default;
void dynamic_vino_lib::PersonReidentification::loadNetwork(
    const std::shared_ptr<Models::PersonReidentificationModel> network) {
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::PersonReidentification::enqueue(
  const cv::Mat& frame, const cv::Rect& input_frame_loc) {
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  if (!dynamic_vino_lib::BaseInference::enqueue<u_int8_t>(
          frame, input_frame_loc, 1, 0, valid_model_->getInputName())) {
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::PersonReidentification::submitRequest() {
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::PersonReidentification::fetchResults() {
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) return false;
  bool found_result = false;
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float* output_values = request->GetBlob(output)->buffer().as<float*>();
  for (int i = 0; i < getResultsLength(); i++) {
    std::vector<float> new_person = std::vector<float>(
      output_values + 256 * i, output_values + 256 * i + 256);
    std::string person_id = findMatchPerson(new_person);
    results_[i].person_id_ = person_id;
    found_result = true;
  }
  if (!found_result) results_.clear();
  return true;
}

float dynamic_vino_lib::PersonReidentification::calcSimilarity(
  const std::vector<float>& person_a, const std::vector<float>& person_b) {
  if (person_a.size() != person_b.size()) {
    throw std::logic_error("cosine similarity can't be called for the vectors of different lengths: "
                           "person_a size = " + std::to_string(person_a.size()) +
                           "person_b size = " + std::to_string(person_b.size()));
  }
  float mul_sum, denom_a, denom_b, value_a, value_b;
  mul_sum = denom_a = denom_b = value_a = value_b = 0;
  for (auto i = 0; i < person_a.size(); i++) {
      value_a = person_a[i];
      value_b = person_b[i];
      mul_sum += value_a * value_b;
      denom_a += value_a * value_a;
      denom_b += value_b * value_b;
  }
  if (denom_a == 0 || denom_b == 0) {
      throw std::logic_error("cosine similarity is not defined whenever one or both "
                             "input vectors are zero-vectors.");
  }
  return mul_sum / (sqrt(denom_a) * sqrt(denom_b));
}

std::string dynamic_vino_lib::PersonReidentification::findMatchPerson(
  const std::vector<float>& new_person) {
  auto size = recorded_persons_.size();
  std::string id = "No.";
  float best_match_sim = 0;
  int best_match_ind = -1;
  for (auto i = 0; i < size; ++i) {
      float cos_sim = calcSimilarity(new_person, recorded_persons_[i]);
      if (cos_sim > best_match_sim) {
        best_match_sim = cos_sim;
        best_match_ind = i;
      }
  }
  std::cout<<"Similarity: "<<best_match_sim<<std::endl;
  if (best_match_sim > match_thresh_) {
    recorded_persons_[best_match_ind] = new_person;
    return id + std::to_string(best_match_ind);
  } else {
    recorded_persons_.push_back(new_person);
    return id + std::to_string(size);
  }
}

const int dynamic_vino_lib::PersonReidentification::getResultsLength() const {
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result*
dynamic_vino_lib::PersonReidentification::getLocationResult(int idx) const {
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::PersonReidentification::getName() const {
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::PersonReidentification::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output) {
  if (output != nullptr) {
    output->accept(results_);
  }
}
