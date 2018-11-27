/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @brief a header file with declaration of ObjectDetectionModel class
 * @file object_detection_model.cpp
 */
#include <string>
#include "dynamic_vino_lib/models/object_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
// Validated Object Detection Network
Models::ObjectDetectionModel::ObjectDetectionModel(const std::string& model_loc,
                                               int input_num, int output_num,
                                               int max_batch_size)
    : BaseModel(model_loc, input_num, output_num, max_batch_size){}
void Models::ObjectDetectionModel::setLayerProperty(
    InferenceEngine::CNNNetReader::Ptr net_reader) {
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
      net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    throw std::logic_error("This sample accepts networks having only one input");
  }
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(
      net_reader->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 1) {
    throw std::logic_error("This sample accepts networks having only one output");
  }
  InferenceEngine::DataPtr& output_data_ptr = output_info_map.begin()->second;
  output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  output_data_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;
}
void Models::ObjectDetectionModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr& net_reader) {
  slog::info << "Checking Object Detection outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info_map(
      net_reader->getNetwork().getOutputsInfo());
  slog::info << "Checking Object Detection outputs ..." << slog::endl;
  if (output_info_map.size() != 1) {
    throw std::logic_error("This sample accepts networks having only one output");
  }
  InferenceEngine::DataPtr& output_data_ptr = output_info_map.begin()->second;
  output_ = output_info_map.begin()->first;
  slog::info << "Checking Object Detection output ... Name=" << output_ << slog::endl;

  const InferenceEngine::CNNLayerPtr output_layer =
      net_reader->getNetwork().getLayerByName(output_.c_str());
  // output layer should have attribute called num_classes
  slog::info << "Checking Object Detection num_classes" << slog::endl;
  if (output_layer->params.find("num_classes") == output_layer->params.end()) {
    throw std::logic_error("Object Detection network output layer (" + output_ +
                           ") should have num_classes integer attribute");
  }
  // class number should be equal to size of label vector
  // if network has default "background" class, fake is used
  const int num_classes = output_layer->GetParamAsInt("num_classes");

  slog::info << "Checking Object Detection output ... num_classes=" << num_classes << slog::endl;
  if (getLabels().size() != num_classes) {
    if (getLabels().size() == (num_classes - 1)) {
      getLabels().insert(getLabels().begin(), "fake");
    } else {
      getLabels().clear();
    }
  }
  // last dimension of output layer should be 7
  const InferenceEngine::SizeVector output_dims =
      output_data_ptr->getTensorDesc().getDims();
  max_proposal_count_ = static_cast<int>(output_dims[2]);
  slog::info << "max proposal count is: " << max_proposal_count_ << slog::endl;
  object_size_ = static_cast<int>(output_dims[3]);
  if (object_size_ != 7) {
    throw std::logic_error(
        "Object Detection network output layer should have 7 as a last "
        "dimension");
  }
  if (output_dims.size() != 4) {
    throw std::logic_error(
        "Object Detection network output dimensions not compatible shoulld be 4, "
        "but was " +
        std::to_string(output_dims.size()));
  }
}
const std::string Models::ObjectDetectionModel::getModelName() const {
  return "Object Detection";
}
