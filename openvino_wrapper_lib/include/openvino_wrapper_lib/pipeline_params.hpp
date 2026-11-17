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
 * @brief a header file with declaration of Pipeline class
 * @file pipeline_params.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__PIPELINE_PARAMS_HPP_
#define OPENVINO_WRAPPER_LIB__PIPELINE_PARAMS_HPP_

#include <openvino_param_lib/param_manager.hpp>
#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "openvino_wrapper_lib/inferences/base_inference.hpp"
#include "openvino_wrapper_lib/inputs/standard_camera.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "opencv2/opencv.hpp"

const char kInputType_Image[] = "Image";
const char kInputType_Video[] = "Video";
const char kInputType_StandardCamera[] = "StandardCamera";
const char kInputType_IpCamera[] = "IpCamera";
const char kInputType_CameraTopic[] = "RealSenseCameraTopic";
const char kInputType_ImageTopic[] = "ImageTopic";
const char kInputType_RealSenseCamera[] = "RealSenseCamera";
const char kInputType_ServiceImage[] = "ServiceImage";

const char kOutputType_RViz[] = "RViz";
const char kOutputType_ImageWindow[] = "ImageWindow";
const char kOutputType_RosTopic[] = "RosTopic";
const char kOutputType_RosService[] = "RosService";

const char kInferType_FaceDetection[] = "FaceDetection";
const char kInferType_AgeGenderRecognition[] = "AgeGenderRecognition";
const char kInferType_EmotionRecognition[] = "EmotionRecognition";
const char kInferType_HeadPoseEstimation[] = "HeadPoseEstimation";
const char kInferType_ObjectDetection[] = "ObjectDetection";
const char kInferType_ObjectSegmentation[] = "ObjectSegmentation";
const char kInferType_ObjectSegmentationMaskrcnn[] = "ObjectSegmentationMaskrcnn";
const char kInferType_ObjectSegmentationInstance[] = "ObjectSegmentationInstance";
const char kInferType_ObjectSegmentationTypeYolo[] = "yolo";
const char kInferType_ObjectSegmentationTypeMaskrcnn[] = "maskrcnn";
const char kInferType_ObjectDetectionTypeSSD[] = "SSD";
const char kInferType_ObjectDetectionTypeYolov5[] = "yolov5";
const char kInferType_ObjectDetectionTypeYolov8[] = "yolov8";
const char kInferType_PersonReidentification[] = "PersonReidentification";
const char kInferType_PersonAttribsDetection[] = "PersonAttribsDetection";
const char kInferType_LandmarksDetection[] = "LandmarksDetection";
const char kInferType_FaceReidentification[] = "FaceReidentification";
const char kInferType_VehicleAttribsDetection[] = "VehicleAttribsDetection";
const char kInferType_LicensePlateDetection[] = "LicensePlateDetection";

/**
 * @class PipelineParams
 * @brief This class is a pipeline parameter management that stores parameters
 * of a given pipeline
 */
class PipelineParams
{
public:
  explicit PipelineParams(const std::string& name);
  explicit PipelineParams(const Params::ParamManager::PipelineRawData& params);
  Params::ParamManager::PipelineRawData getPipeline(const std::string& name);
  PipelineParams& operator=(const Params::ParamManager::PipelineRawData& params);
  void update();
  void update(const Params::ParamManager::PipelineRawData& params);
  bool isOutputTo(std::string& name);
  bool isGetFps();
  std::string findFilterConditions(const std::string& input, const std::string& output);

private:
  Params::ParamManager::PipelineRawData params_;
};

#endif  // OPENVINO_WRAPPER_LIB__PIPELINE_PARAMS_HPP_
