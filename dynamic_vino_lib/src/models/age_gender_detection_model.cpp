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
 * @brief a header file with declaration of AgeGenderDetectionModel class
 * @file age_gender_detection_model.cpp
 */
#include <utility>
#include <string>

#include "dynamic_vino_lib/models/age_gender_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"

// Validated Age Gender Classification Network
Models::AgeGenderDetectionModel::AgeGenderDetectionModel(
  const std::string & model_loc,
  int max_batch_size)
: BaseModel(model_loc, max_batch_size)
{
}
bool Models::AgeGenderDetectionModel::updateLayerProperty(
   )
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  InferenceEngine::InputsDataMap input_info_map(getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    slog::warn << "This model seems not Age-Gender-like, which should have only one input,"
      <<" but we got " << std::to_string(input_info_map.size()) << "inputs"
      << slog::endl;
    return false;
  }
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::FP32);
  input_info->setLayout(InferenceEngine::Layout::NCHW);
  addInputInfo("input", input_info_map.begin()->first);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(getNetwork().getOutputsInfo());
  if (output_info_map.size() != 2) {
    // throw std::logic_error("Age/Gender Recognition network should have two output layers");
    slog::warn << "This model seems not Age-gender like, which should have and only have 2"
      " outputs, but we got " << std::to_string(output_info_map.size()) << "outputs"
      << slog::endl;
    return false;
  }
  auto it = output_info_map.begin();
  InferenceEngine::DataPtr age_output_ptr = (it++)->second;
  InferenceEngine::DataPtr gender_output_ptr = (it++)->second;

  //Check More Configuration:
  if (gender_output_ptr->getCreatorLayer().lock()->type == "Convolution") {
    std::swap(age_output_ptr, gender_output_ptr);
  }
  if (age_output_ptr->getCreatorLayer().lock()->type != "Convolution") {
    slog::err << "In Age Gender network, age layer ("
      << age_output_ptr->getCreatorLayer().lock()->name
      << ") should be a Convolution, but was: "
      << age_output_ptr->getCreatorLayer().lock()->type << slog::endl;
    return false;
  }
  if (gender_output_ptr->getCreatorLayer().lock()->type != "SoftMax") {
    slog::err <<"In Age Gender network, gender layer ("
      << gender_output_ptr->getCreatorLayer().lock()->name
      << ") should be a SoftMax, but was: "
      << gender_output_ptr->getCreatorLayer().lock()->type
      << slog::endl;
    return false;
  }
  slog::info << "Age layer: " << age_output_ptr->getCreatorLayer().lock()->name << slog::endl;
  slog::info << "Gender layer: " << gender_output_ptr->getCreatorLayer().lock()->name << slog::endl;

  age_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  age_output_ptr->setLayout(InferenceEngine::Layout::NCHW);
  gender_output_ptr->setPrecision(InferenceEngine::Precision::FP32);
  gender_output_ptr->setLayout(InferenceEngine::Layout::NCHW);

  //output_age_ = age_output_ptr->name;
  addOutputInfo("age", age_output_ptr->getName());
  //output_gender_ = gender_output_ptr->name;
  addOutputInfo("gender", gender_output_ptr->getName());

  printAttribute();
  return true;
}

const std::string Models::AgeGenderDetectionModel::getModelCategory() const
{
  return "Age Gender Detection";
}
