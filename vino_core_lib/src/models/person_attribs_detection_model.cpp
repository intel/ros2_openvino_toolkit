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
 * @brief a header file with declaration of PersonAttribsDetectionModel class
 * @file person_attribs_detection_model.cpp
 */
#include <string>
#include "vino_core_lib/models/person_attribs_detection_model.hpp"
#include "vino_core_lib/slog.hpp"
// Validated Person Attributes Detection Network
Models::PersonAttribsDetectionModel::PersonAttribsDetectionModel(
  const std::string & model_loc, int max_batch_size)
: BaseModel(model_loc, max_batch_size) {}

bool Models::PersonAttribsDetectionModel::updateLayerProperty(
  InferenceEngine::CNNNetwork& net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  InferenceEngine::InputsDataMap input_info_map(
    net_reader.getInputsInfo());
  if (input_info_map.size() != 1) {
    throw std::logic_error("Person Attribs topology should have only one input");
  }
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  addInputInfo("input", input_info_map.begin()->first);

  slog::info << "Checking OUTPUTs for model " << getModelName() << slog::endl;
  InferenceEngine::OutputsDataMap output_info_map(
    net_reader.getOutputsInfo());
  if (output_info_map.size() != 3) {
    throw std::logic_error("Person Attribs Network expects networks having 3 output");
  }
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;

  auto output_iter = output_info_map.begin();
  InferenceEngine::DataPtr attribute_output_ptr = (output_iter++)->second;
  InferenceEngine::DataPtr top_output_ptr = (output_iter++)->second;
  InferenceEngine::DataPtr bottom_output_ptr = (output_iter++)->second;
    
  addOutputInfo("attributes_output_", attribute_output_ptr->getName());
  //output_gender_ = gender_output_ptr->name;
  addOutputInfo("top_output_", top_output_ptr->getName());
  addOutputInfo("bottom_output_", bottom_output_ptr->getName());
  printAttribute();
  return true;
}

const std::string Models::PersonAttribsDetectionModel::getModelCategory() const
{
  return "Person Attributes Detection";
}
