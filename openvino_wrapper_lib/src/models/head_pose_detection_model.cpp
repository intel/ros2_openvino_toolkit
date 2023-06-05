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
 * @brief a header file with declaration of HeadPoseDetectionModel class
 * @file head_pose_detection_model.cpp
 */

#include <map>
#include <string>

#include "openvino_wrapper_lib/models/head_pose_detection_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// Validated Head Pose Network
Models::HeadPoseDetectionModel::HeadPoseDetectionModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::HeadPoseDetectionModel::updateLayerProperty
(std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  auto input_info_map = model->inputs();
  if (input_info_map.size() != 1) {
    slog::warn << "This model should have only one input, but we got"
      << std::to_string(input_info_map.size()) << "inputs"
      << slog::endl; 
    return false;
  }

  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  const ov::Layout input_tensor_layout{"NCHW"};
  input_info.tensor().
    set_element_type(ov::element::u8).
    set_layout(input_tensor_layout);
  addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);

  // set output property
  auto output_info_map = model->outputs();
  std::vector<std::string> outputs_name;
  for (auto & output_item : output_info_map) {
    std::string output_tensor_name_ = output_item.get_any_name();
    const ov::Layout output_tensor_layout{"NC"};
    ppp.output(output_tensor_name_).
      tensor().
      set_element_type(ov::element::f32).
      set_layout(output_tensor_layout);
    outputs_name.push_back(output_tensor_name_);
  }

  model = ppp.build();
  ov::set_batch(model, getMaxBatchSize());

  for (const std::string& outName : {output_angle_r_, output_angle_p_, output_angle_y_}) {
    if (find(outputs_name.begin(), outputs_name.end(), outName) == outputs_name.end()) {
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
