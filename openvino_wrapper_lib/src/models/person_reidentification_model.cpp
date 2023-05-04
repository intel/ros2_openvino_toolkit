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
 * @brief a header file with declaration of PersonReidentificationModel class
 * @file person_reidentification_model.cpp
 */
#include <string>
#include "openvino_wrapper_lib/models/person_reidentification_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
// Validated Person Reidentification Network
Models::PersonReidentificationModel::PersonReidentificationModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

bool Models::PersonReidentificationModel::updateLayerProperty(
  std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking Inputs for Model" << getModelName() << slog::endl;
  auto input_info_map = model->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_ = input_info_map[0].get_any_name();
  const ov::Layout input_tensor_layout{"NCHW"};
  ppp.input(input_).
    tensor().
    set_element_type(ov::element::u8).
    set_layout(input_tensor_layout);

  // set output property
  auto output_info_map = model->outputs();
  output_ = output_info_map[0].get_any_name();

  model = ppp.build();
  ov::set_batch(model, getMaxBatchSize());

  return true;
}

const std::string Models::PersonReidentificationModel::getModelCategory() const
{
  return "Person Reidentification";
}
