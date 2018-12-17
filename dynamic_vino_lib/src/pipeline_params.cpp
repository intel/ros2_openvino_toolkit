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
 * @brief a header file with declaration of Pipeline class
 * @file pipeline.cpp
 */

#include <memory>
#include <string>
#include <utility>

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline_params.hpp"

const std::string kInputType_Image = "Image";
const std::string kInputType_Video = "Video";
const std::string kInputType_StandardCamera = "StandardCamera";
const std::string kInputType_CameraTopic = "RealSenseCameraTopic";
const std::string kInputType_RealSenseCamera = "RealSenseCamera";

const std::string kOutputTpye_RViz = "RViz";
const std::string kOutputTpye_ImageWindow = "ImageWindow";
const std::string kOutputTpye_RosTopic = "RosTopic";

const std::string kInferTpye_FaceDetection = "FaceDetection";
const std::string kInferTpye_AgeGenderRecognition = "AgeGenderRecognition";
const std::string kInferTpye_EmotionRecognition = "EmotionRecognition";
const std::string kInferTpye_HeadPoseEstimation = "HeadPoseEstimation";
const std::string kInferTpye_ObjectDetection = "ObjectDetection";
const std::string kInferTpye_ObjectSegmentation = "ObjectSegmentation";

PipelineParams::PipelineParams(const std::string& name) { params_.name = name; }

PipelineParams::PipelineParams(
    const Params::ParamManager::PipelineParams& params) {
  params_ = params;
}

PipelineParams& PipelineParams::operator=(
    const Params::ParamManager::PipelineParams& params) {
  params_.name = params.name;
  params_.infers = params.infers;
  params_.inputs = params.inputs;
  params_.outputs = params.outputs;
  params_.connects = params.connects;

  return *this;
}

Params::ParamManager::PipelineParams PipelineParams::getPipeline(
    const std::string& name) {
  return Params::ParamManager::getInstance().getPipeline(name);
}

void PipelineParams::update() {
  if (!params_.name.empty()) {
    params_ = getPipeline(params_.name);
  }
}

void PipelineParams::update(
    const Params::ParamManager::PipelineParams& params) {
  params_ = params;
}

bool PipelineParams::isOutputTo(std::string& output) {
  if (std::find(params_.outputs.begin(), params_.outputs.end(), output) !=
      params_.outputs.end()) {
    return true;
  }
  return false;
}

bool PipelineParams::isGetFps() {
  /**< Only "Image" input can't computing FPS >**/
  if (params_.inputs.size() == 0) {
    return false;
  }
  return std::find(params_.inputs.begin(), params_.inputs.end(),
                   kInputType_Image) == params_.inputs.end();
}
