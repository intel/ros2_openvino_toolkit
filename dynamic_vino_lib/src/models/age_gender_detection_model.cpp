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
 * @brief a header file with declaration of AgeGenderDetectionModel class
 * @file age_gender_detection_model.cpp
 */
#include <utility>
#include <string>

#include "dynamic_vino_lib/models/age_gender_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"


// Validated Age Gender Classification Network
Models::AgeGenderDetectionModel::AgeGenderDetectionModel(
    const std::string& model_loc, int input_num, int output_num,
    int max_batch_size)
    : BaseModel(model_loc, input_num, output_num, max_batch_size){}

void Models::AgeGenderDetectionModel::setLayerProperty(
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
  auto it = output_info_map.begin();
  InferenceEngine::DataPtr age_output_ptr = (it++)->second;
  InferenceEngine::DataPtr gender_output_ptr = (it++)->second;
  age_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  age_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  gender_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  gender_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_age_ = age_output_ptr->name;
  output_gender_ = gender_output_ptr->name;
}

void Models::AgeGenderDetectionModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr& net_reader) {
  slog::info << "Checking Age Gender Detection outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(
      net_reader->getNetwork().getOutputsInfo());
  auto it = output_info.begin();
  InferenceEngine::DataPtr age_output_ptr = (it++)->second;
  InferenceEngine::DataPtr gender_output_ptr = (it++)->second;
  // output layer of age should be Convolution type
  if (gender_output_ptr->getCreatorLayer().lock()->type == "Convolution") {
    std::swap(age_output_ptr, gender_output_ptr);
  }
  if (age_output_ptr->getCreatorLayer().lock()->type != "Convolution") {
    throw std::logic_error("In Age Gender network, age layer (" +
                           age_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a Convolution, but was: " +
                           age_output_ptr->getCreatorLayer().lock()->type);
  }
  if (gender_output_ptr->getCreatorLayer().lock()->type != "SoftMax") {
    throw std::logic_error("In Age Gender network, gender layer (" +
                           gender_output_ptr->getCreatorLayer().lock()->name +
                           ") should be a SoftMax, but was: " +
                           gender_output_ptr->getCreatorLayer().lock()->type);
  }
  slog::info << "Age layer: " << age_output_ptr->getCreatorLayer().lock()->name
             << slog::endl;
  slog::info << "Gender layer: "
             << gender_output_ptr->getCreatorLayer().lock()->name << slog::endl;
}

const std::string Models::AgeGenderDetectionModel::getModelName() const {
  return "Age Gender Detection";
}
