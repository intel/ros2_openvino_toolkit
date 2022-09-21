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
 * @brief a header file with declaration of PersonReidentificationModel class
 * @file person_reidentification_model.cpp
 */
#include <string>
#include "dynamic_vino_lib/models/person_reidentification_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
// Validated Person Reidentification Network
Models::PersonReidentificationModel::PersonReidentificationModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}
/*
void Models::PersonReidentificationModel::setLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
    net_reader->getNetwork().getInputsInfo());
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(
    net_reader->getNetwork().getOutputsInfo());
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;
}

void Models::PersonReidentificationModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & net_reader) {}

const std::string Models::PersonReidentificationModel::getModelCategory() const
{
  return "Person Reidentification";
}
*/
bool Models::PersonReidentificationModel::updateLayerProperty(
  std::shared_ptr<ov::Model>& net_reader)
{
  slog::info << "Checking Inputs for Model" << getModelName() << slog::endl;
  auto input_info_map = net_reader->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(net_reader);
  std::string input_tensor_name_ = net_reader->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  const ov::Layout input_tensor_layout{"NHWC"};
  input_info.tensor().
              set_element_type(ov::element::u8).
              set_layout(input_tensor_layout);
  addInputInfo("input", input_tensor_name_);
  // set output property
  auto output_info_map = net_reader -> outputs();
  std::string output_tensor_name_ = net_reader->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  addOutputInfo("output", output_tensor_name_);
  return true;
}

const std::string Models::PersonReidentificationModel::getModelCategory() const
{
  return "Person Reidentification";
}
