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
 * @brief a header file with declaration of EmotionsDetection class and
 * EmotionsResult class
 * @file emotions_recognition.cpp
 */

#include <memory>
#include <string>

#include "dynamic_vino_lib/inferences/emotions_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// EmotionsResult
dynamic_vino_lib::EmotionsResult::EmotionsResult(const cv::Rect& location)
    : Result(location) {}

// Emotions Detection
dynamic_vino_lib::EmotionsDetection::EmotionsDetection()
    : dynamic_vino_lib::BaseInference(){}

dynamic_vino_lib::EmotionsDetection::~EmotionsDetection() = default;

void dynamic_vino_lib::EmotionsDetection::loadNetwork(
    const std::shared_ptr<Models::EmotionDetectionModel> network) {
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::EmotionsDetection::enqueue(
    const cv::Mat& frame, const cv::Rect& input_frame_loc) {
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  bool succeed = dynamic_vino_lib::BaseInference::enqueue<float>(
      frame, input_frame_loc, 1, getResultsLength(),
      valid_model_->getInputName());
  if (!succeed) {
    slog::err << "Failed enqueue Emotion frame." << slog::endl;
    // TODO(weizhi): throw an error here
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::EmotionsDetection::submitRequest() {
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::EmotionsDetection::fetchResults() {
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) return false;
  int label_length = static_cast<int>(valid_model_->getLabels().size());
  std::string output_name = valid_model_->getOutputName();
  InferenceEngine::Blob::Ptr emotions_blob =
      getEngine()->getRequest()->GetBlob(output_name);
  /** emotions vector must have the same size as number of channels
      in model output. Default output format is NCHW so we check index 1 */

  int64 num_of_channels = emotions_blob->getTensorDesc().getDims().at(1);
  if (num_of_channels != label_length) {
    slog::err << "Output size (" << num_of_channels
              << ") of the Emotions Recognition network is not equal "
              << "to used emotions vector size (" << label_length << ")"
              << slog::endl;
    throw std::logic_error("Output size (" + std::to_string(num_of_channels) +
                           ") of the Emotions Recognition network is not equal "
                           "to used emotions vector size (" +
                           std::to_string(label_length) + ")");
  }

  /** we identify an index of the most probable emotion in output array
      for idx image to return appropriate emotion name */
  auto emotions_values = emotions_blob->buffer().as<float*>();
  for (int idx = 0; idx < results_.size(); ++idx) {
    auto output_idx_pos = emotions_values + label_length*idx;
    int64 max_prob_emotion_idx =
        std::max_element(output_idx_pos, output_idx_pos + label_length) -
        output_idx_pos;
    results_[idx].label_ = valid_model_->getLabels()[max_prob_emotion_idx];
  }

  return true;
}

const int dynamic_vino_lib::EmotionsDetection::getResultsLength() const {
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result*
dynamic_vino_lib::EmotionsDetection::getLocationResult(int idx) const {
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::EmotionsDetection::getName() const {
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::EmotionsDetection::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput>& output) {
  if (output != nullptr) {
    output->accept(results_);
  }
}
