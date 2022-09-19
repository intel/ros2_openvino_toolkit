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
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::HeadPoseDetectionModel::updateLayerProperty
(std::shared_ptr<ov::Model>& net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  auto input_info_map = net_reader -> inputs();
  // InferenceEngine::InputsDataMap input_info_map(net_reader.getInputsInfo());
  if (input_info_map.size() != 1) {
    slog::warn << "This model should have only one input, but we got"
      << std::to_string(input_info_map.size()) << "inputs"
      << slog::endl; 
    return false;
  }

  // InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  // input_info->setPrecision(InferenceEngine::Precision::U8);
  // input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // addInputInfo("input", input_info_map.begin()->first);
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(net_reader);
  std::string input_tensor_name_ = net_reader->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  const ov::Layout input_tensor_layout{"NHWC"};
  input_info.tensor().
              set_element_type(ov::element::u8).
              set_layout(input_tensor_layout);
  addInputInfo("input", input_tensor_name_);

  // set output property
  // InferenceEngine::OutputsDataMap output_info_map(net_reader.getOutputsInfo());
  auto output_info_map = net_reader -> outputs();
  {
  // for (auto & output : output_info_map) {
    // output.second->setPrecision(InferenceEngine::Precision::FP32);
    // output.second->setLayout(InferenceEngine::Layout::NC);
  std::string output_tensor_name_ = net_reader->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  const ov::Layout output_tensor_layout{"NC"};
  output_info.tensor().
              set_element_type(ov::element::u8).
              set_layout(output_tensor_layout);
  addOutputInfo("output", output_tensor_name_);
  }

  // for (const std::string& outName : {output_angle_r_, output_angle_p_, output_angle_y_}) {
  //   if (output_info_map.find(output_tensor_name_) == output_info_map.end()) {
  //     throw std::logic_error("There is no " + outName + " output in Head Pose Estimation network");
  //   } else {
  //     addOutputInfo(outName, outName);
  //   }
  // }

  printAttribute();
  return true;
}

const std::string Models::HeadPoseDetectionModel::getModelCategory() const
{
  return "Head Pose Network";
}
