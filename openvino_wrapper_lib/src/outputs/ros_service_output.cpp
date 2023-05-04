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
 * @brief a header file with declaration of RosServiceOutput class
 * @file ros_service_output.cpp
 */

#include <vector>
#include <string>
#include <memory>
#include "openvino_wrapper_lib/outputs/ros_service_output.hpp"
#include "cv_bridge/cv_bridge.h"

void Outputs::RosServiceOutput::setServiceResponse(
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  if (detected_objects_topic_ != nullptr && detected_objects_topic_->objects_vector.size() > 0) {
    response->objects.objects_vector = detected_objects_topic_->objects_vector;
  } else if (faces_topic_ != nullptr && faces_topic_->objects_vector.size() > 0) {
    response->objects.objects_vector = faces_topic_->objects_vector;
  }
}

void Outputs::RosServiceOutput::setResponseForFace(
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  if (faces_topic_ != nullptr && faces_topic_->objects_vector.size() > 0) {
    response->objects.objects_vector = faces_topic_->objects_vector;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  std::shared_ptr<object_msgs::srv::AgeGenderSrv::Response> response)
{
  if (age_gender_topic_ != nullptr) {
    response->age_gender.objects = age_gender_topic_->objects;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  std::shared_ptr<object_msgs::srv::EmotionSrv::Response> response)
{
  if (emotions_topic_ != nullptr) {
    response->emotion.emotions = emotions_topic_->emotions;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  std::shared_ptr<object_msgs::srv::HeadPoseSrv::Response> response)
{
  if (headpose_topic_ != nullptr) {
    response->headpose.headposes = headpose_topic_->headposes;
  }
}

void Outputs::RosServiceOutput::setServiceResponse(
  std::shared_ptr<object_msgs::srv::People::Response> response)
{
  slog::info << "in People::Response ...";
  if (faces_topic_ != nullptr) {
    slog::info << "[FACES],";
    response->persons.faces = faces_topic_->objects_vector;
  } else if (detected_objects_topic_ != nullptr) {
    slog::info << "[FACES(objects)],";
    response->persons.faces = detected_objects_topic_->objects_vector;
  }
  if (age_gender_topic_ != nullptr) {
    slog::info << "[AGE_GENDER],";
    response->persons.agegenders = age_gender_topic_->objects;
  }
  if (emotions_topic_ != nullptr) {
    slog::info << "[EMOTION],";
    response->persons.emotions = emotions_topic_->emotions;
  }
  if (headpose_topic_ != nullptr) {
    slog::info << "[HEADPOSE],";
    response->persons.headposes = headpose_topic_->headposes;
  }
  slog::info << "DONE!" << slog::endl;
}

void Outputs::RosServiceOutput::clearData()
{
  faces_topic_ = nullptr;
  detected_objects_topic_ = nullptr;
  age_gender_topic_ = nullptr;
  emotions_topic_ = nullptr;
  headpose_topic_ = nullptr;
}
