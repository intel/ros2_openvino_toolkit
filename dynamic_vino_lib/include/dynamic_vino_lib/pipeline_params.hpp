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
 * @file pipeline_params.hpp
 */
#ifndef DYNAMIC_VINO_LIB__PIPELINE_PARAM_HPP_
#define DYNAMIC_VINO_LIB__PIPELINE_PARAM_HPP_

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/inferences/base_inference.hpp"
#include "dynamic_vino_lib/inputs/standard_camera.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "opencv2/opencv.hpp"

extern const std::string kInputType_Image;
extern const std::string kInputType_Video;
extern const std::string kInputType_StandardCamera;
extern const std::string kInputType_CameraTopic;
extern const std::string kInputType_RealSenseCamera;

extern const std::string kOutputTpye_RViz;
extern const std::string kOutputTpye_ImageWindow;
extern const std::string kOutputTpye_RosTopic;

extern const std::string kInferTpye_FaceDetection;
extern const std::string kInferTpye_AgeGenderRecognition;
extern const std::string kInferTpye_EmotionRecognition;
extern const std::string kInferTpye_HeadPoseEstimation;
extern const std::string kInferTpye_ObjectDetection;
extern const std::string kInferTpye_ObjectSegmentation;

/**
 * @class PipelineParams
 * @brief This class is a pipeline parameter management that stores parameters
 * of a given pipeline
 */
class PipelineParams {
 public:
  PipelineParams(const std::string& name);
  PipelineParams(const Params::ParamManager::PipelineParams& params);
  static Params::ParamManager::PipelineParams getPipeline(
      const std::string& name);
  PipelineParams& operator=(const Params::ParamManager::PipelineParams& params);
  void update();
  void update(const Params::ParamManager::PipelineParams& params);
  bool isOutputTo(std::string& name);
  bool isGetFps();

 private:
  Params::ParamManager::PipelineParams params_;
};

#endif  // DYNAMIC_VINO_LIB__PIPELINE_PARAM_HPP_
