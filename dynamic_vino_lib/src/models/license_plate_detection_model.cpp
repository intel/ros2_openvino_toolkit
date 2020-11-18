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
 * @brief a realization file with declaration of LicensePlateDetectionModel class
 * @file license_plate_detection_model.cpp
 */
#include <string>
#include "dynamic_vino_lib/models/license_plate_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
// Validated Vehicle Attributes Detection Network
Models::LicensePlateDetectionModel::LicensePlateDetectionModel(
  const std::string & model_loc, int max_batch_size)
: BaseModel(model_loc, max_batch_size) {}
/*
void Models::LicensePlateDetectionModel::setLayerProperty(
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
  seq_input_ = (++input_info_map.begin())->first;
  output_ = output_info_map.begin()->first;
}

void Models::LicensePlateDetectionModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & net_reader)
{
  InferenceEngine::InputsDataMap input_info_map(
    net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 2) {
    throw std::logic_error("Vehicle Attribs topology should have only two inputs");
  }
  auto sequence_input = (++input_info_map.begin());
  if (sequence_input->second->getTensorDesc().getDims()[0] != getMaxSequenceSize()) {
    throw std::logic_error("License plate detection max sequence size dismatch");
  }
  InferenceEngine::OutputsDataMap output_info_map(
    net_reader->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 1) {
    throw std::logic_error("Vehicle Attribs Network expects networks having one output");
  }
}*/
bool Models::LicensePlateDetectionModel::updateLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  InferenceEngine::InputsDataMap input_info_map(
    net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 2) {
    throw std::logic_error("Vehicle Attribs topology should have only two inputs");
  }
  auto sequence_input = (++input_info_map.begin());
  if (sequence_input->second->getTensorDesc().getDims()[0] != getMaxSequenceSize()) {
    throw std::logic_error("License plate detection max sequence size dismatch");
  }
  InferenceEngine::OutputsDataMap output_info_map(
    net_reader->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 1) {
    throw std::logic_error("Vehicle Attribs Network expects networks having one output");
  }

  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);

  // set input and output layer name
  input_ = input_info_map.begin()->first;
  seq_input_ = (++input_info_map.begin())->first;
  output_ = output_info_map.begin()->first;

  return true;
}

const std::string Models::LicensePlateDetectionModel::getModelCategory() const
{
  return "Vehicle Attributes Detection";
}
