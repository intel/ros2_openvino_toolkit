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
 * @brief a header file with declaration of FaceDetectionModel class
 * @file face_detection_model.cpp
 */

#include <string>

#include "dynamic_vino_lib/models/face_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"

// Validated Face Detection Network
Models::FaceDetectionModel::FaceDetectionModel(
  const std::string & model_loc, int input_num,
  int output_num, int max_batch_size)
: ObjectDetectionModel(model_loc, input_num, output_num, max_batch_size)
{
}

void Models::FaceDetectionModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & net_reader)
{
  slog::info << "Checking Face Detection inputs" << slog::endl;
  InferenceEngine::InputsDataMap input_info_map(net_reader->getNetwork().getOutputsInfo());
  if (input_info_map.size() != 1) {
    slog::err << "Face Detection network should have only one input, but we got "
      << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    throw std::logic_error("Face Detection network should have only one input");
  }

  slog::info << "Checking Face Detection outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info_map(net_reader->getNetwork().getOutputsInfo());
  slog::info << "Checking Face Detection outputs ..." << slog::endl;
  if (output_info_map.size() != 1) {
    throw std::logic_error("This sample accepts networks having only one output");
  }
  InferenceEngine::DataPtr & output_data_ptr = output_info_map.begin()->second;
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
  const InferenceEngine::SizeVector output_dims = output_data_ptr->getTensorDesc().getDims();
  max_proposal_count_ = static_cast<int>(output_dims[2]);
  slog::info << "max proposal count is: " << max_proposal_count_ << slog::endl;

  auto object_size = static_cast<int>(output_dims[3]);
  if (object_size != 7) {
    throw std::logic_error("Object Detection network output layer should have 7 as a last "
            "dimension");
  }
  setObjectSize(object_size);

  if (output_dims.size() != 4) {
    throw std::logic_error("Object Detection network output dimensions not compatible shoulld be "
            "4, "
            "but was " +
            std::to_string(output_dims.size()));
  }
}

const std::string Models::FaceDetectionModel::getModelName() const
{
  return "Face Detection";
}
