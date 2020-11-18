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
 * @brief a realization file with declaration of VehicleAttribsDetectionModel class
 * @file vehicle_attribs_detection_model.cpp
 */
#include <string>
#include "dynamic_vino_lib/models/vehicle_attribs_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
// Validated Vehicle Attributes Detection Network
Models::VehicleAttribsDetectionModel::VehicleAttribsDetectionModel(
  const std::string & model_loc, int max_batch_size)
: BaseModel(model_loc, max_batch_size) {}
/*
void Models::VehicleAttribsDetectionModel::setLayerProperty(
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
  auto output_iter = output_info_map.begin();
  color_output_ = (output_iter++)->second->name;
  type_output_ = (output_iter++)->second->name;
}

void Models::VehicleAttribsDetectionModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & net_reader)
{
  InferenceEngine::InputsDataMap input_info_map(
    net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    throw std::logic_error("Vehicle Attribs topology should have only one input");
  }
  InferenceEngine::OutputsDataMap output_info_map(
    net_reader->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 2) {
    throw std::logic_error("Vehicle Attribs Network expects networks having two outputs");
  }
}
*/
bool Models::VehicleAttribsDetectionModel::updateLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
    // set input property
  InferenceEngine::InputsDataMap input_info_map(
    net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    throw std::logic_error("Vehicle Attribs topology should have only one input");
  }
  InferenceEngine::OutputsDataMap output_info_map(
    net_reader->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 2) {
    throw std::logic_error("Vehicle Attribs Network expects networks having two outputs");
  }

  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
 
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  auto output_iter = output_info_map.begin();
  // color_output_ = (output_iter++)->second->name;
  // type_output_ = (output_iter++)->second->name;
  InferenceEngine::DataPtr color_output_ptr = (output_iter++)->second;
  InferenceEngine::DataPtr type_output_ptr = (output_iter++)->second;
    
  addOutputInfo("color_output_", color_output_ptr->getName());
  //output_gender_ = gender_output_ptr->name;
  addOutputInfo("type_output_", type_output_ptr->getName());

  printAttribute();
  return true;
}

const std::string Models::VehicleAttribsDetectionModel::getModelCategory() const
{
  return "Vehicle Attributes Detection";
}

