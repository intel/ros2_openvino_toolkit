// Copyright (c) 2020-2022 Intel Corporation
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
 * @brief implementation of SSDModelAttr class
 * @file ssd_model_attr.cpp
 */

#include <string>
 
#include "openvino_wrapper_lib/models/attributes/base_attribute.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// Validated Face Detection Network
Models::SSDModelAttr::SSDModelAttr(
  const std::string model_name)
: ModelAttribute(model_name)
{
}

bool Models::SSDModelAttr::updateLayerProperty(
  const std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  auto input_info_map = model->inputs();
  if (input_info_map.size() != 1) {
    slog::warn << "This model seems not SSDNet-like, SSDnet has only one input, but we got "
      << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    return false;
  }
  
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  input_info.tensor().set_element_type(ov::element::u8);
  addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);

  ov::Shape input_dims = input_info_map[0].get_shape();
  setInputHeight(input_dims[2]);
  setInputWidth(input_dims[3]);

  slog::info << "Checking OUTPUTs for model " << getModelName() << slog::endl;
   auto outputs_info = model->outputs();
  if (outputs_info.size() != 1) {
    slog::warn << "This model seems not SSDNet-like! We got " 
      << std::to_string(outputs_info.size()) << "outputs, but SSDnet has only one."
      << slog::endl;
    return false;
  }

  ov::preprocess::OutputInfo& output_info = ppp.output();
  addOutputInfo(ModelAttribute::DefaultOutputName, model->output().get_any_name());
  slog::info << "Checking Object Detection output ... Name=" << model->output().get_any_name()
    << slog::endl;

  output_info.tensor().set_element_type(ov::element::f32);

///TODO: double check this part: BEGIN
#if(0) ///
  const InferenceEngine::CNNLayerPtr output_layer =
    model->getNetwork().getLayerByName(output_info_map.begin()->first.c_str());
  // output layer should have attribute called num_classes
  slog::info << "Checking Object Detection num_classes" << slog::endl;
  if (output_layer->params.find("num_classes") == output_layer->params.end()) {
    slog::warn << "This model's output layer (" << output_info_map.begin()->first
      << ") should have num_classes integer attribute" << slog::endl;
    return false;
  }
  // class number should be equal to size of label vector
  // if network has default "background" class, fake is used
  const int num_classes = output_layer->GetParamAsInt("num_classes");
#endif
#if 0
  slog::info << "Checking Object Detection output ... num_classes=" << num_classes << slog::endl;
  if (getLabels().size() != num_classes) {
    if (getLabels().size() == (num_classes - 1)) {
      getLabels().insert(getLabels().begin(), "fake");
    } else {
      getLabels().clear();
    }
  }
#endif
  ///TODO: double check this part: END

  // last dimension of output layer should be 7
  auto outputsDataMap = model->outputs();
  auto & data = outputsDataMap[0];
  ov::Shape output_dims = data.get_shape();
  setMaxProposalCount(static_cast<int>(output_dims[2]));
  slog::info << "max proposal count is: " << getMaxProposalCount() << slog::endl;

  auto object_size = static_cast<int>(output_dims[3]);
  if (object_size != 7) {
    slog::warn << "This model is NOT SSDNet-like, whose output data for each detected object"
      << "should have 7 dimensions, but was " << std::to_string(object_size)
      << slog::endl;
    return false;
  }
  setObjectSize(object_size);

  if (output_dims.size() != 4) {
    slog::warn << "This model is not SSDNet-like, output dimensions shoulld be 4, but was"
      << std::to_string(output_dims.size()) << slog::endl;
    return false;
  }

  printAttribute();
  slog::info << "This model is SSDNet-like, Layer Property updated!" << slog::endl;
  return true;
}
