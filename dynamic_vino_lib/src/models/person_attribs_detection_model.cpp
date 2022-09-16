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
 * @brief a header file with declaration of PersonAttribsDetectionModel class
 * @file person_attribs_detection_model.cpp
 */
#include <string>
#include <openvino/openvino.hpp>
#include "dynamic_vino_lib/models/person_attribs_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
// Validated Person Attributes Detection Network
Models::PersonAttribsDetectionModel::PersonAttribsDetectionModel(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

bool Models::PersonAttribsDetectionModel::updateLayerProperty(
  std::shared_ptr<ov::Model> net_reader)
{ 
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;


  // InferenceEngine::InputsDataMap input_info_map(
  //   net_reader.getInputsInfo());
  // if (input_info_map.size() != 1) {
  //   throw std::logic_error("Person Attribs topology should have only one input");
  auto network = net_reader;
  inputs_info_ = network->inputs();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(network);
  input_tensor_name_ = network->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);

  ov::Shape input_tensor_shape = network->input().get_shape();
  if (input_tensor_shape.size() != 1) {
    throw std::logic_error("Person Attribs topology should have only one input");
  }


  // InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  // input_info->setPrecision(InferenceEngine::Precision::U8);
  // input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // addInputInfo("input", input_info_map.begin()->first);
  const ov::Layout tensor_layout{"NHWC"};
  input_info.tensor().
              set_element_type(ov::element::u8).
              set_layout(tensor_layout);
  // ppp.input().preprocess().
  //           convert_element_type(ov::element::f32).
  //           convert_layout("NCHW");
  // input_info.model().set_layout("NCHW");
  // ppp.input().preprocess().resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR);


  slog::info << "Checking OUTPUTs for model " << getModelName() << slog::endl;
  // InferenceEngine::OutputsDataMap output_info_map(
  //   net_reader.getOutputsInfo());
  outputs_info_ = network->outputs();
  output_tensor_name_ = network->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  ov::Shape output_tensor_shape = network->output().get_shape();
  if (output_tensor_shape.size() != 3) {
    throw std::logic_error("Person Attribs Network expects networks having 3 output");
  }
  // network = ppp.build
// //   input_ = input_info_map.begin()->first;
// //   output_ = output_info_map.begin()->first;
// //   auto output_iter = output_info_map.begin();
// //   InferenceEngine::DataPtr attribute_output_ptr = (output_iter++)->second;
// //   InferenceEngine::DataPtr top_output_ptr = (output_iter++)->second;
// //   InferenceEngine::DataPtr bottom_output_ptr = (output_iter++)->second;
    
// //   addOutputInfo("attributes_output_", attribute_output_ptr->getName());
// //   //output_gender_ = gender_output_ptr->name;
// //   addOutputInfo("top_output_", top_output_ptr->getName());
// //   addOutputInfo("bottom_output_", bottom_output_ptr->getName());


  printAttribute();
  return true;
}

const std::string Models::PersonAttribsDetectionModel::getModelCategory() const
{
  return "Person Attributes Detection";
}
