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
 * @brief a header file with declaration of EmotionDetectionModel class
 * @file emotion_detection_model.cpp
 */
#include <string>

#include "dynamic_vino_lib/models/emotion_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"


// Validated Emotions Detection Network
Models::EmotionDetectionModel::EmotionDetectionModel(
    const std::string& model_loc, int input_num, int output_num,
    int max_batch_size)
    : BaseModel(model_loc, input_num, output_num, max_batch_size){}

void Models::EmotionDetectionModel::setLayerProperty(
    InferenceEngine::CNNNetReader::Ptr net_reader) {
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
      net_reader->getNetwork().getInputsInfo());
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::FP32);
  input_info->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(
      net_reader->getNetwork().getOutputsInfo());
  InferenceEngine::DataPtr& output_data_ptr = output_info_map.begin()->second;
  output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  output_data_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;
}

void Models::EmotionDetectionModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr& net_reader) {
  slog::info << "Checking Emotions Detection outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(
      net_reader->getNetwork().getOutputsInfo());
  InferenceEngine::DataPtr emotions_output_ptr = output_info.begin()->second;
  // output layer should be SoftMax type
  if (emotions_output_ptr->getCreatorLayer().lock()->type != "SoftMax") {
    throw std::logic_error("In Emotions Recognition network, Emotion layer (" +
                           emotions_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a SoftMax, but was: " +
                           emotions_output_ptr->getCreatorLayer().lock()->type);
  }
  slog::info << "Emotions layer: "
             << emotions_output_ptr->getCreatorLayer().lock()->name
             << slog::endl;
}

const std::string Models::EmotionDetectionModel::getModelName() const {
  return "Emotions Detection";
}
