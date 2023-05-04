// Copyright (c) 2018-2022 Intel Corporation
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
 * @brief a header file with declaration of LandmarksDetectionModel class
 * @file landmarks_detection_model.cpp
 */
#include <string>
#include "openvino_wrapper_lib/models/landmarks_detection_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
// Validated Landmarks Detection Network
Models::LandmarksDetectionModel::LandmarksDetectionModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

void Models::LandmarksDetectionModel::setLayerProperty(
  InferenceEngine::CNNNetwork& model)
{
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
    model.getInputsInfo());
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(
    model.getOutputsInfo());
  InferenceEngine::DataPtr & output_data_ptr = output_info_map.begin()->second;
  output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  output_data_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;
}

void Models::LandmarksDetectionModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & model)
{
  InferenceEngine::InputsDataMap input_info_map(
    model->getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    throw std::logic_error("Landmarks Detection topology should have only one input");
  }
  InferenceEngine::OutputsDataMap output_info_map(
    model->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 1) {
    throw std::logic_error("Landmarks Detection Network expects networks having one output");
  }
}

const std::string Models::LandmarksDetectionModel::getModelCategory() const
{
  return "Landmarks Detection";
}
