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
 * @brief a header file with declaration of EmotionDetectionModel class
 * @file emotion_detection_model.cpp
 */
#include <string>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/models/emotion_detection_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"


// Validated Emotions Detection Network
Models::EmotionDetectionModel::EmotionDetectionModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::EmotionDetectionModel::updateLayerProperty
(std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  inputs_info_ = model->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);

  ov::Shape input_tensor_shape = model->input().get_shape();
  if (inputs_info_.size() != 1) {
    slog::warn << "This model seems not Emotion-detection-model-like, which should have only one input, but we got"
      << std::to_string(input_tensor_shape.size()) << "inputs"
      << slog::endl;
    return false;
  }

  addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);
  const ov::Layout tensor_layout{"NHWC"};
  input_info.tensor().
    set_element_type(ov::element::f32).
    set_layout(tensor_layout);

  // set output property
  outputs_info_ = model->outputs();
  output_tensor_name_ = model->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  if (outputs_info_.size() != 1) {
    slog::warn << "This model should have and only have 1 output, but we got "
      << std::to_string(outputs_info_.size()) << "outputs"
      << slog::endl;
    return false;
  }

  model = ppp.build();
  ov::set_batch(model, getMaxBatchSize());
  addOutputInfo(ModelAttribute::DefaultOutputName, output_tensor_name_);

  printAttribute();
  return true; 
}

const std::string Models::EmotionDetectionModel::getModelCategory() const
{
  return "Emotions Detection";
}
