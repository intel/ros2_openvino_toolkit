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
 * @brief a header file with declaration of RosTopicOutput class
 * @file ros_topic_output.cpp
 */

#include <vector>
#include <string>
#include <memory>
#include "dynamic_vino_lib/outputs/ros_service_output.hpp"
#include "cv_bridge/cv_bridge.h"

// Outputs::RosServiceOutput::RosServiceOutput()

void Outputs::RosServiceOutput::feedFrame(const cv::Mat & frame)
{
  frame_ = frame.clone();
}

void Outputs::RosServiceOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectDetectionResult> & results)
{
  objects_.clear();
  for (auto & r : results) {
    auto loc = r.getLocation();
    object_.roi.x_offset = loc.x;
    object_.roi.y_offset = loc.y;
    object_.roi.width = loc.width;
    object_.roi.height = loc.height;
    object_.object.object_name = r.getLabel();
    object_.object.probability = r.getConfidence();
    objects_.push_back(object_);
  }
}

void Outputs::RosServiceOutput::accept(
  const std::vector<dynamic_vino_lib::FaceDetectionResult> & results)
{
  for (auto r : results) {
    auto loc = r.getLocation();
    face_.roi.x_offset = loc.x;
    face_.roi.y_offset = loc.y;
    face_.roi.width = loc.width;
    face_.roi.height = loc.height;
    face_.object.object_name = r.getLabel();
    face_.object.probability = r.getConfidence();
    faces_.push_back(face_);
  }
}

void Outputs::RosServiceOutput::accept(
  const std::vector<dynamic_vino_lib::EmotionsResult> & results)
{
  for (auto r : results) {
    auto loc = r.getLocation();
    emotion_.roi.x_offset = loc.x;
    emotion_.roi.y_offset = loc.y;
    emotion_.roi.width = loc.width;
    emotion_.roi.height = loc.height;
    emotion_.emotion = r.getLabel();
    emotions_.push_back(emotion_);
  }
}

void Outputs::RosServiceOutput::accept(
  const std::vector<dynamic_vino_lib::AgeGenderResult> & results)
{
  for (auto r : results) {
    auto loc = r.getLocation();
    ag_.roi.x_offset = loc.x;
    ag_.roi.y_offset = loc.y;
    ag_.roi.width = loc.width;
    ag_.roi.height = loc.height;
    ag_.age = r.getAge();
    auto male_prob = r.getMaleProbability();
    if (male_prob > 0.5) {
      ag_.gender = "Male";
      ag_.gender_confidence = male_prob;
    } else {
      ag_.gender = "Female";
      ag_.gender_confidence = 1.0 - male_prob;
    }
    ags_.push_back(ag_);
  }
}

void Outputs::RosServiceOutput::accept(
  const std::vector<dynamic_vino_lib::HeadPoseResult> & results)
{
  for (auto r : results) {
    auto loc = r.getLocation();
    hp_.roi.x_offset = loc.x;
    hp_.roi.y_offset = loc.y;
    hp_.roi.width = loc.width;
    hp_.roi.height = loc.height;
    hp_.yaw = r.getAngleY();
    hp_.pitch = r.getAngleP();
    hp_.roll = r.getAngleR();
    hps_.push_back(hp_);
  }
}


void Outputs::RosServiceOutput::setResponseForObject(
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  response->objects.objects_vector = objects_;
}

void Outputs::RosServiceOutput::setResponseForFace(
  std::shared_ptr<object_msgs::srv::DetectObject::Response> response)
{
  response->objects.objects_vector = faces_;
}

void Outputs::RosServiceOutput::setResponseForAgeGender(
  std::shared_ptr<people_msgs::srv::AgeGender::Response> response)
{
  response->age_gender.objects = ags_;
}

void Outputs::RosServiceOutput::setResponseForEmotion(
  std::shared_ptr<people_msgs::srv::Emotion::Response> response)
{
  response->emotion.emotions = emotions_;
}

void Outputs::RosServiceOutput::setResponseForHeadPose(
  std::shared_ptr<people_msgs::srv::HeadPose::Response> response)
{
  response->headpose.headposes = hps_;
}

/**
 * TODO: implement the value gain
 */
std_msgs::msg::Header Outputs::RosServiceOutput::getHeader()
{
  std_msgs::msg::Header header;
  header.frame_id = "default_camera";

  std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
  int64 ns = tp.time_since_epoch().count();
  header.stamp.sec = ns / 1000000000;
  header.stamp.nanosec = ns % 1000000000;
  return header;
}
