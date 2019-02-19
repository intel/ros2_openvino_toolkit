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

#include "dynamic_vino_lib/services/frame_processing_server.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <vino_param_lib/param_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <map>

#include "dynamic_vino_lib/pipeline_manager.hpp"
#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/inputs/base_input.hpp"
#include "dynamic_vino_lib/inputs/image_input.hpp"
#include "dynamic_vino_lib/slog.hpp"

namespace vino_service
{
FrameProcessingServer::FrameProcessingServer(
  const std::string service_name,
  const std::string config_path)
: Node("node_with_service")
{
  Params::ParamManager::getInstance().parse(config_path);
  Params::ParamManager::getInstance().print();
  auto pcommon = Params::ParamManager::getInstance().getCommon();
  auto pipelines = Params::ParamManager::getInstance().getPipelines();

  if (pipelines.size() < 1) {
    throw std::logic_error("Pipeline parameters should be set!");
  }

  for (auto & p : pipelines) {
    PipelineManager::getInstance().createPipeline(p);
  }

  for (auto & p : pipelines) {
    for (unsigned int i = 0; i < p.infers.size(); i++) {
      if (!p.infers[i].name.compare("FaceDetection")) {
        face_service_ = create_service<object_msgs::srv::DetectObject>(
          "/detect_face", std::bind(&FrameProcessingServer::cbFaceDetection, this,
          std::placeholders::_1, std::placeholders::_2));
      } else if (!p.infers[i].name.compare("AgeGenderRecognition")) {
        age_gender_service_ = create_service<people_msgs::srv::AgeGender>(
          "/detect_age_gender", std::bind(&FrameProcessingServer::cbAgeGenderRecognition, this,
          std::placeholders::_1, std::placeholders::_2));
      } else if (!p.infers[i].name.compare("EmotionRecognition")) {
        emotion_service_ = create_service<people_msgs::srv::Emotion>(
          "/detect_emotion", std::bind(&FrameProcessingServer::cbEmotionRecognition, this,
          std::placeholders::_1, std::placeholders::_2));
      } else if (!p.infers[i].name.compare("HeadPoseEstimation")) {
        head_pose_service_ = create_service<people_msgs::srv::HeadPose>(
          "/detect_head_pose", std::bind(&FrameProcessingServer::cbHeadPoseRecognition, this,
          std::placeholders::_1, std::placeholders::_2));
      } else if (!p.infers[i].name.compare("ObjectDetection")) {
        object_service_ = create_service<object_msgs::srv::DetectObject>(
          "/detect_object", std::bind(&FrameProcessingServer::cbObjectDetection, this,
          std::placeholders::_1, std::placeholders::_2));
      }
    }
  }
}

void FrameProcessingServer::cbFaceDetection(
  const std::shared_ptr<object_msgs::srv::DetectObject::Request> request,
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  /*
std::map<std::string, PipelineManager::PipelineData> pipelines_ =
PipelineManager::getInstance().getPipelines();
for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
  PipelineManager::PipelineData& p = pipelines_[it->second.params.name.c_str()];
  //p.pipeline->runService(request->image_path);
  //auto output_handle = p.pipeline->getOutputHandle();

  for (auto& pair : output_handle) {
    if (pair.first.compare("FaceDetection")) {
      pair.second -> setResponse(response);
      response->objects.inference_time_ms = 11.11;
    }
  }
  */
  // p.pipeline->runService(request->image_path);
  response->objects.inference_time_ms = 11.11;
  //}
}

void FrameProcessingServer::cbAgeGenderRecognition(
  const std::shared_ptr<people_msgs::srv::AgeGender::Request> request,
  std::shared_ptr<people_msgs::srv::AgeGender::Response> response)
{
  std::cout << "inside cb" << std::endl;

  std::map<std::string, PipelineManager::PipelineData> pipelines_ =
    PipelineManager::getInstance().getPipelines();
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    PipelineManager::PipelineData & p = pipelines_[it->second.params.name.c_str()];
    p.pipeline->runService(request->image_path);
    auto output_handle = p.pipeline->getOutputHandle();

    for (auto & pair : output_handle) {
      if (pair.first.compare("AgeGenderRecognition")) {
        pair.second->setResponse(response);
      }
    }
    // p.pipeline->runService(request->image_path);
  }
}

void FrameProcessingServer::cbEmotionRecognition(
  const std::shared_ptr<people_msgs::srv::Emotion::Request> request,
  std::shared_ptr<people_msgs::srv::Emotion::Response> response)
{
  std::cout << "inside cb" << std::endl;

  std::map<std::string, PipelineManager::PipelineData> pipelines_ =
    PipelineManager::getInstance().getPipelines();
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    PipelineManager::PipelineData & p = pipelines_[it->second.params.name.c_str()];
    p.pipeline->runService(request->image_path);
    auto output_handle = p.pipeline->getOutputHandle();

    for (auto & pair : output_handle) {
      if (pair.first.compare("EmotionRecognition")) {
        pair.second->setResponse(response);
      }
    }
    // p.pipeline->runService(request->image_path);
  }
}

void FrameProcessingServer::cbHeadPoseRecognition(
  const std::shared_ptr<people_msgs::srv::HeadPose::Request> request,
  std::shared_ptr<people_msgs::srv::HeadPose::Response> response)
{
  std::cout << "inside cb" << std::endl;

  std::map<std::string, PipelineManager::PipelineData> pipelines_ =
    PipelineManager::getInstance().getPipelines();
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    PipelineManager::PipelineData & p = pipelines_[it->second.params.name.c_str()];
    p.pipeline->runService(request->image_path);
    auto output_handle = p.pipeline->getOutputHandle();

    for (auto & pair : output_handle) {
      if (pair.first.compare("HeadPoseEstimation")) {
        pair.second->setResponse(response);
      }
    }
    // p.pipeline->runService(request->image_path);
  }
}

void FrameProcessingServer::cbObjectDetection(
  const std::shared_ptr<object_msgs::srv::DetectObject::Request> request,
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  std::map<std::string, PipelineManager::PipelineData> pipelines_ =
    PipelineManager::getInstance().getPipelines();
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    PipelineManager::PipelineData & p = pipelines_[it->second.params.name.c_str()];
    p.pipeline->runService(request->image_path);
    auto output_handle = p.pipeline->getOutputHandle();

    for (auto & pair : output_handle) {
      if (!pair.first.compare("RosService")) {
        pair.second->setResponse(response);
      }
    }
  }
}

}  // namespace vino_service
