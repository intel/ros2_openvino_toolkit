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
 * @brief a header file with declaration of HeadPoseDetectionModel class
 * @file head_pose_detection_model.cpp
 */

#include <map>
#include <string>

#include "dynamic_vino_lib/models/head_pose_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"


// Validated Head Pose Network
Models::HeadPoseDetectionModel::HeadPoseDetectionModel(
    const std::string& model_loc, int input_num, int output_num,
    int max_batch_size)
    : BaseModel(model_loc, input_num, output_num, max_batch_size){}

void Models::HeadPoseDetectionModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr& net_reader) {
  slog::info << "Checking Head Pose network outputs" << slog::endl;
  InferenceEngine::OutputsDataMap outputInfo(
      net_reader->getNetwork().getOutputsInfo());
  std::map<std::string, bool> layerNames = {{output_angle_r_, false},
                                            {output_angle_p_, false},
                                            {output_angle_y_, false}};

  for (auto&& output : outputInfo) {
    InferenceEngine::CNNLayerPtr layer =
        output.second->getCreatorLayer().lock();
    if (layerNames.find(layer->name) == layerNames.end()) {
      throw std::logic_error("Head Pose network output layer unknown: " +
                             layer->name + ", should be " + output_angle_r_ +
                             " or " + output_angle_p_ + " or " +
                             output_angle_y_);
    }
    if (layer->type != "FullyConnected") {
      throw std::logic_error("Head Pose network output layer (" + layer->name +
                             ") has invalid type: " + layer->type +
                             ", should be FullyConnected");
    }
    auto fc = dynamic_cast<InferenceEngine::FullyConnectedLayer*>(layer.get());
    if (fc->_out_num != 1) {
      throw std::logic_error("Head Pose network output layer (" + layer->name +
                             ") has invalid out-size=" +
                             std::to_string(fc->_out_num) + ", should be 1");
    }
    layerNames[layer->name] = true;
  }
}

void Models::HeadPoseDetectionModel::setLayerProperty(
    InferenceEngine::CNNNetReader::Ptr net_reader) {
  // set input property
  InferenceEngine::InputsDataMap input_info_map(
      net_reader->getNetwork().getInputsInfo());

  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  input_ = input_info_map.begin()->first;

  // set output property
  InferenceEngine::OutputsDataMap output_info_map(
      net_reader->getNetwork().getOutputsInfo());

  for (auto& output : output_info_map){
    output.second->setPrecision(InferenceEngine::Precision::FP32);
    output.second->setLayout(InferenceEngine::Layout::NC);
  }
}

const std::string Models::HeadPoseDetectionModel::getModelName() const {
  return "Head Pose Network";
}
