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
 * @brief a header file with declaration of AgeGenderDetectionModel class
 * @file age_gender_detection_model.cpp
 */
#include <utility>
#include <string>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/models/age_gender_detection_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// Validated Age Gender Classification Network
Models::AgeGenderDetectionModel::AgeGenderDetectionModel(
  const std::string & label_loc,
  const std::string & model_loc,
  int max_batch_size)
: BaseModel(label_loc,model_loc, max_batch_size)
{
}
bool Models::AgeGenderDetectionModel::updateLayerProperty(
  std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  inputs_info_ = model->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);

  ov::Shape input_tensor_shape = model->input().get_shape();
  if (inputs_info_.size() != 1) {
    slog::warn << "This model seems not Age-Gender-like, which should have only one input, but we got"
      << std::to_string(input_tensor_shape.size()) << "inputs"
      << slog::endl;
    return false;
  }

  addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);
  const ov::Layout tensor_layout{"NCHW"};
  input_info.tensor().
    set_element_type(ov::element::f32).
    set_layout(tensor_layout);

  // set output property
  outputs_info_ = model->outputs();
  if (outputs_info_.size() != 2) {
    slog::warn << "This model seems not Age-Gender-like, which should have and only have 2 outputs, but we got"
      << std::to_string(outputs_info_.size()) << "outputs"
      << slog::endl;
    return false;
  }

#if(0) ///
  //Check More Configuration:
  if (gender_output_ptr->getCreatorLayer().lock()->type == "Convolution") {
    std::swap(age_output_ptr, gender_output_ptr);
  }
  if (age_output_ptr->getCreatorLayer().lock()->type != "Convolution") {
    slog::err << "In Age Gender network, age layer ("
      << age_output_ptr->getCreatorLayer().lock()->name
      << ") should be a Convolution, but was: "
      << age_output_ptr->getCreatorLayer().lock()->type << slog::endl;
    return false;
  }
  if (gender_output_ptr->getCreatorLayer().lock()->type != "SoftMax") {
    slog::err <<"In Age Gender network, gender layer ("
      << gender_output_ptr->getCreatorLayer().lock()->name
      << ") should be a SoftMax, but was: "
      << gender_output_ptr->getCreatorLayer().lock()->type
      << slog::endl;
    return false;
  }
  slog::info << "Age layer: " << age_output_ptr->getCreatorLayer().lock()->name << slog::endl;
  slog::info << "Gender layer: " << gender_output_ptr->getCreatorLayer().lock()->name << slog::endl;
#endif

 auto age_output_info = outputs_info_[1];
 ppp.output(age_output_info.get_any_name()).
    tensor().
    set_element_type(ov::element::f32).
    set_layout(tensor_layout);
  auto gender_output_info = outputs_info_[0];
  ppp.output(gender_output_info.get_any_name()).
    tensor().
    set_element_type(ov::element::f32).
    set_layout(tensor_layout);

  model = ppp.build();
  ov::set_batch(model, getMaxBatchSize());

  addOutputInfo("age", age_output_info.get_any_name());
  addOutputInfo("gender", gender_output_info.get_any_name());
  printAttribute();
  return true;
}

const std::string Models::AgeGenderDetectionModel::getModelCategory() const
{
  return "Age Gender Detection";
}
