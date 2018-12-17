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
 * @brief a header file with declaration of RosTopicOutput class
 * @file ros_topic_output.cpp
 */

#include <vector>
#include <string>
#include <memory>
#include "dynamic_vino_lib/outputs/ros_topic_output.hpp"
#include "cv_bridge/cv_bridge.h"


Outputs::RosTopicOutput::RosTopicOutput() {
  // rmw_qos_profile_t qos = rmw_qos_profile_default;
  // qos.depth = 10;
  // qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  node_ = rclcpp::Node::make_shared("topic_publisher");
  pub_segmented_object_ = node_->create_publisher<people_msgs::msg::ObjectsInMasks>(
    "/openvino_toolkit/segmented_obejcts", 16);
  pub_detected_object_ = node_->create_publisher<object_msgs::msg::ObjectsInBoxes>(
    "/openvino_toolkit/detected_objects", 16);
  pub_face_ = node_->create_publisher<object_msgs::msg::ObjectsInBoxes>(
      "/openvino_toolkit/faces", 16);
  pub_emotion_ = node_->create_publisher<people_msgs::msg::EmotionsStamped>(
      "/openvino_toolkit/emotions", 16);
  pub_age_gender_ = node_->create_publisher<people_msgs::msg::AgeGenderStamped>(
    "/openvino_toolkit/age_genders", 16);
  pub_headpose_ = node_->create_publisher<people_msgs::msg::HeadPoseStamped>(
    "/openvino_toolkit/headposes", 16);
  emotions_topic_ = nullptr;
  detected_objects_topic_ = nullptr;
  faces_topic_ = nullptr;
  age_gender_topic_ = nullptr;
  headpose_topic_ = nullptr;
  segmented_objects_topic_ = nullptr;
}

void Outputs::RosTopicOutput::feedFrame(const cv::Mat& frame) {frame_ = frame.clone();}


void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectSegmentationResult>& results) {
  segmented_objects_topic_ = std::make_shared<people_msgs::msg::ObjectsInMasks>();
  people_msgs::msg::ObjectInMask object;
  for (auto& r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    object.roi.x_offset = loc.x;
    object.roi.y_offset = loc.y;
    object.roi.width = loc.width;
    object.roi.height = loc.height;
    object.object_name = r.getLabel();
    object.probability = r.getConfidence();
    cv::Mat mask = r.getMask();
    for (int h = 0; h < mask.size().height; ++h)
      for (int w = 0; w < mask.size().width; ++w)
          object.mask_array.push_back(mask.at<float>(h, w));
    segmented_objects_topic_->objects_vector.push_back(object);
  }
}


void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectDetectionResult>& results) {
  detected_objects_topic_ = std::make_shared<object_msgs::msg::ObjectsInBoxes>();
  object_msgs::msg::ObjectInBox object;
  for (auto& r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    object.roi.x_offset = loc.x;
    object.roi.y_offset = loc.y;
    object.roi.width = loc.width;
    object.roi.height = loc.height;
    object.object.object_name = r.getLabel();
    object.object.probability = r.getConfidence();
    detected_objects_topic_->objects_vector.push_back(object);
  }
}


void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::FaceDetectionResult>& results) {
  faces_topic_ = std::make_shared<object_msgs::msg::ObjectsInBoxes>();

  object_msgs::msg::ObjectInBox face;
  for (auto r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    face.roi.x_offset = loc.x;
    face.roi.y_offset = loc.y;
    face.roi.width = loc.width;
    face.roi.height = loc.height;
    face.object.object_name = r.getLabel();
    face.object.probability = r.getConfidence();
    faces_topic_->objects_vector.push_back(face);
    detected_objects_topic_->objects_vector.push_back(face);
  }
}

void Outputs::RosTopicOutput::accept(
    const std::vector<dynamic_vino_lib::EmotionsResult>& results) {
  emotions_topic_ = std::make_shared<people_msgs::msg::EmotionsStamped>();

  people_msgs::msg::Emotion emotion;
  for (auto r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    emotion.roi.x_offset = loc.x;
    emotion.roi.y_offset = loc.y;
    emotion.roi.width = loc.width;
    emotion.roi.height = loc.height;
    emotion.emotion = r.getLabel();
    emotions_topic_->emotions.push_back(emotion);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::AgeGenderResult>& results) {
  age_gender_topic_ = std::make_shared<people_msgs::msg::AgeGenderStamped>();

  people_msgs::msg::AgeGender ag;
  for (auto r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    ag.roi.x_offset = loc.x;
    ag.roi.y_offset = loc.y;
    ag.roi.width = loc.width;
    ag.roi.height = loc.height;
    ag.age = r.getAge();
    auto male_prob = r.getMaleProbability();
    if (male_prob > 0.5){
      ag.gender = "Male";
      ag.gender_confidence = male_prob;
    } else {
      ag.gender = "Female";
      ag.gender_confidence = 1.0 - male_prob;
    }
    age_gender_topic_->objects.push_back(ag);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::HeadPoseResult>& results) {
  headpose_topic_ = std::make_shared<people_msgs::msg::HeadPoseStamped>();

  people_msgs::msg::HeadPose hp;
  for (auto r : results) {
    auto loc = r.getLocation();
    hp.roi.x_offset = loc.x;
    hp.roi.y_offset = loc.y;
    hp.roi.width = loc.width;
    hp.roi.height = loc.height;
    hp.yaw = r.getAngleY();
    hp.pitch = r.getAngleP();
    hp.roll = r.getAngleR();
    headpose_topic_->headposes.push_back(hp);
  }
}

void Outputs::RosTopicOutput::handleOutput() {
  auto header = getHeader();
  if (segmented_objects_topic_ != nullptr) {
    // slog::info << "publishing faces outputs." << slog::endl;
    segmented_objects_topic_->header = header;
    pub_segmented_object_->publish(segmented_objects_topic_);
    segmented_objects_topic_ = nullptr;
  }
  if (detected_objects_topic_ != nullptr) {
    // slog::info << "publishing faces outputs." << slog::endl;
    detected_objects_topic_->header = header;
    pub_detected_object_->publish(detected_objects_topic_);
    detected_objects_topic_ = nullptr;
  }
  if (faces_topic_ != nullptr) {
    // slog::info << "publishing faces outputs." << slog::endl;
    faces_topic_->header = header;
    pub_face_->publish(faces_topic_);
    faces_topic_ = nullptr;
  }
  if (emotions_topic_ != nullptr) {
    // slog::info << "publishing emotions outputs." << slog::endl;
    emotions_topic_->header = header;
    pub_emotion_->publish(emotions_topic_);
    emotions_topic_ = nullptr;
  }
  if (age_gender_topic_ != nullptr) {
    // slog::info << "publishing age gender outputs." << slog::endl;
    age_gender_topic_->header = header;
    pub_age_gender_->publish(age_gender_topic_);
    age_gender_topic_ = nullptr;
  }
  if (headpose_topic_ != nullptr) {
    headpose_topic_->header = header;
    pub_headpose_->publish(headpose_topic_);
    headpose_topic_ = nullptr;
  }
}

/**
 * TODO: implement the value gain
 */
std_msgs::msg::Header Outputs::RosTopicOutput::getHeader() {
  std_msgs::msg::Header header;
  header.frame_id = "default_camera";

  std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
  int64 ns = tp.time_since_epoch().count();
  header.stamp.sec = ns/1000000000;
  header.stamp.nanosec = ns%1000000000;
  return header;
}
