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
 * @brief a realization file with declaration of VehicleAttribsDetectionModel class
 * @file vehicle_attribs_detection_model.cpp
 */
#include <string>
#include "openvino_wrapper_lib/models/vehicle_attribs_detection_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
// Validated Vehicle Attributes Detection Network
Models::VehicleAttribsDetectionModel::VehicleAttribsDetectionModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

bool Models::VehicleAttribsDetectionModel::updateLayerProperty(
  std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  auto input_info_map = model->inputs();
  if (input_info_map.size() != 1) {
    throw std::logic_error("Vehicle Attribs topology should have only one input");
  }

  auto output_info_map = model->outputs();
  if (output_info_map.size() != 2) {
    throw std::logic_error("Vehicle Attribs Network expects networks having two outputs");
  }

  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  const ov::Layout tensor_layout{"NCHW"};
  input_info.tensor().
    set_element_type(ov::element::u8).
    set_layout(tensor_layout);
  model = ppp.build();

  addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);
 
  // set input and output layer name
  addOutputInfo("color_output_", output_info_map[1].get_any_name());
  addOutputInfo("type_output_", output_info_map[0].get_any_name());

  printAttribute();
  return true;
}

const std::string Models::VehicleAttribsDetectionModel::getModelCategory() const
{
  return "Vehicle Attributes Detection";
}

