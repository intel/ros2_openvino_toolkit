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
 * @brief a header file with declaration of EmotionDetectionModel class
 * @file emotion_detection_model.cpp
 */
#include <string>
#include <openvino/openvino.hpp>
#include "dynamic_vino_lib/models/emotion_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"

// Validated Emotions Detection Network
Models::EmotionDetectionModel::EmotionDetectionModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::EmotionDetectionModel::updateLayerProperty
(std::shared_ptr<ov::Model>& net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  // set input property
  // InferenceEngine::InputsDataMap input_info_map(net_reader.getInputsInfo());
  // if (input_info_map.size() != 1) {
  //   slog::warn << "This model seems not Age-Gender-like, which should have only one input,"
  //     <<" but we got " << std::to_string(input_info_map.size()) << "inputs"
  //     << slog::endl;
  auto network = net_reader;
  inputs_info_ = network->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(network);
  input_tensor_name_ = network->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  ov::Shape input_tensor_shape = network->input().get_shape();
  slog::debug<<"input size"<<input_tensor_shape.size()<<slog::endl;
  if (input_tensor_shape.size() != 1) {
    slog::warn << "This model seems not Emotion-detection-model-like, which should have only one input, but we got"
      << std::to_string(input_tensor_shape.size()) << "inputs"
      << slog::endl;
    return false;
  }

  // InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  // input_info->setPrecision(InferenceEngine::Precision::FP32);
  // input_info->setLayout(InferenceEngine::Layout::NCHW);
  // addInputInfo("input", input_info_map.begin()->first);
  const ov::Layout tensor_layout{"NHWC"};
  input_info.tensor().
              set_element_type(ov::element::f32).
              set_layout(tensor_layout);

  // set output property
  // InferenceEngine::OutputsDataMap output_info_map(net_reader.getOutputsInfo());
  // if (output_info_map.size() != 1) {
  //   // throw std::logic_error("Age/Gender Recognition network should have two output layers");
  //   slog::warn << "This model should have and only have 1 output, but we got "
  //     << std::to_string(output_info_map.size()) << "outputs" << slog::endl;
  //   return false;
  outputs_info_ = network->outputs();
  output_tensor_name_ = network->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  ov::Shape output_tensor_shape = network->output().get_shape();
  if (output_tensor_shape.size() != 1) {
    slog::warn << "This model should have and only have 1 output, but we got "
      << std::to_string(output_tensor_shape.size()) << "outputs"
      << slog::endl;
    return false;
  }

  ///InferenceEngine::DataPtr & output_data_ptr = output_info_map.begin()->second;
  ///slog::info << "Emotions layer: " << output_data_ptr->getCreatorLayer().lock()->name <<
  ///  slog::endl;
  ///output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  ///output_data_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // addOutputInfo("output", output_info_map.begin()->first);

  printAttribute();
  return true; ///verifyOutputLayer(output_data_ptr);
}

// bool Models::EmotionDetectionModel::verifyOutputLayer(const InferenceEngine::DataPtr & ptr)
// {
// ///  if (ptr->getCreatorLayer().lock()->type != "SoftMax") {
// ///    slog::err <<"In Emotion network, gender layer ("
// ///      << ptr->getCreatorLayer().lock()->name
// ///      << ") should be a SoftMax, but was: "
// ///      << ptr->getCreatorLayer().lock()->type
// ///      << slog::endl;
// ///    return false;
// ///  }

//   return true;
// }

const std::string Models::EmotionDetectionModel::getModelCategory() const
{
  return "Emotions Detection";
}
