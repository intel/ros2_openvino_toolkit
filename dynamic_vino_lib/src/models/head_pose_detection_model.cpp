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
 * @brief a header file with declaration of HeadPoseDetectionModel class
 * @file head_pose_detection_model.cpp
 */

#include <map>
#include <string>

#include "dynamic_vino_lib/models/head_pose_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"

// Validated Head Pose Network
Models::HeadPoseDetectionModel::HeadPoseDetectionModel(
  const std::string & model_loc, int max_batch_size)
: BaseModel(model_loc, max_batch_size)
{
}

bool Models::HeadPoseDetectionModel::updateLayerProperty
(InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  InferenceEngine::InputsDataMap input_info_map(getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    slog::warn << "This model should have only one input, but we got"
      << std::to_string(input_info_map.size()) << "inputs"
      << slog::endl;
    return false;
  }
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  addInputInfo("input", input_info_map.begin()->first);

  // set output property
  InferenceEngine::OutputsDataMap output_info_map(getNetwork().getOutputsInfo());
  for (auto & output : output_info_map) {
    output.second->setPrecision(InferenceEngine::Precision::FP32);
    output.second->setLayout(InferenceEngine::Layout::NC);
  }

  for (const std::string& outName : {output_angle_r_, output_angle_p_, output_angle_y_}) {
    if (output_info_map.find(outName) == output_info_map.end()) {
      throw std::logic_error("There is no " + outName + " output in Head Pose Estimation network");
    } else {
      addOutputInfo(outName, outName);
    }
  }

  printAttribute();
  return true;
}

const std::string Models::HeadPoseDetectionModel::getModelCategory() const
{
  return "Head Pose Network";
}
