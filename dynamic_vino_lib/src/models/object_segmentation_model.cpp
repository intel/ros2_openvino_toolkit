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
 * @brief a header file with declaration of ObjectSegmentationModel class
 * @file object_detection_model.cpp
 */
#include <string>
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
// Validated Object Detection Network
Models::ObjectSegmentationModel::ObjectSegmentationModel(
  const std::string& model_loc, int input_num, int output_num, int max_batch_size)
    : BaseModel(model_loc, input_num, output_num, max_batch_size){}

void Models::ObjectSegmentationModel::setLayerProperty(
    InferenceEngine::CNNNetReader::Ptr net_reader) {
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
      net_reader->getNetwork().getInputsInfo());
  auto inputInfoItem = *input_info_map.begin();
  inputInfoItem.second->setPrecision(InferenceEngine::Precision::U8);
  auto network = net_reader->getNetwork();
  network.addOutput(std::string("detection_output"), 0);
  network.setBatchSize(1);
  slog::info << "Batch size is "
             << std::to_string(net_reader->getNetwork().getBatchSize()) << slog::endl;
  InferenceEngine::OutputsDataMap output_info_map(
      net_reader->getNetwork().getOutputsInfo());
  for (auto & item : output_info_map) {
      item.second->setPrecision(InferenceEngine::Precision::FP32);
  }
  auto output_ptr = output_info_map.begin();
  input_ = input_info_map.begin()->first;
  detection_output_ = output_ptr->first;
  mask_output_ = (++output_ptr)->first;
}

void Models::ObjectSegmentationModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr& net_reader) {
  const InferenceEngine::CNNLayerPtr output_layer =
      net_reader->getNetwork().getLayerByName("detection_output");
  const int num_classes = output_layer->GetParamAsInt("num_classes");
  slog::info << "Checking Object Segmentation output ... num_classes=" << num_classes << slog::endl;
  if (getLabels().size() != num_classes) {
    if (getLabels().size() == (num_classes - 1)) {
      getLabels().insert(getLabels().begin(), "fake");
    } else {
      getLabels().clear();
    }
  }
}

const std::string Models::ObjectSegmentationModel::getModelName() const {
  return "Object Segmentation";
}
