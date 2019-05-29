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
#include "dynamic_vino_lib/outputs/ros_topic_output.hpp"
#include "dynamic_vino_lib/pipeline_params.hpp"
#include "dynamic_vino_lib/pipeline.hpp"
#include "cv_bridge/cv_bridge.h"

Outputs::RosTopicOutput::RosTopicOutput(std::string output_name):
  BaseOutput(output_name)
{
  // rmw_qos_profile_t qos = rmw_qos_profile_default;
  // qos.depth = 10;
  // qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // qos.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  node_ = rclcpp::Node::make_shared(output_name + "_topic_publisher");
  pub_landmarks_ = node_->create_publisher<people_msgs::msg::LandmarkStamped>(
    "/openvino_toolkit/" + output_name_ + "/detected_landmarks", 16);
  pub_face_reid_ = node_->create_publisher<people_msgs::msg::ReidentificationStamped>(
    "/openvino_toolkit/" + output_name_ + "/reidentified_faces", 16);
  pub_person_attribs_ = node_->create_publisher<people_msgs::msg::PersonAttributeStamped>(
    "/openvino_toolkit/" + output_name_ + "/person_attributes", 16);
  pub_person_reid_ = node_->create_publisher<people_msgs::msg::ReidentificationStamped>(
    "/openvino_toolkit/" + output_name_ + "/reidentified_persons", 16);
  pub_segmented_object_ = node_->create_publisher<people_msgs::msg::ObjectsInMasks>(
    "/openvino_toolkit/" + output_name_ + "/segmented_obejcts", 16);
  pub_detected_object_ = node_->create_publisher<object_msgs::msg::ObjectsInBoxes>(
    "/openvino_toolkit/" + output_name_ + "/detected_objects", 16);
  pub_face_ =
    node_->create_publisher<object_msgs::msg::ObjectsInBoxes>(
      "/openvino_toolkit/" + output_name_ + "/faces", 16);
  pub_emotion_ =
    node_->create_publisher<people_msgs::msg::EmotionsStamped>(
      "/openvino_toolkit/" + output_name_ + "/emotions", 16);
  pub_age_gender_ = node_->create_publisher<people_msgs::msg::AgeGenderStamped>(
    "/openvino_toolkit/" + output_name_ + "/age_genders", 16);
  pub_headpose_ =
    node_->create_publisher<people_msgs::msg::HeadPoseStamped>(
      "/openvino_toolkit/" + output_name_ + "/headposes", 16);
  emotions_topic_ = nullptr;
  detected_objects_topic_ = nullptr;
  faces_topic_ = nullptr;
  age_gender_topic_ = nullptr;
  headpose_topic_ = nullptr;
  segmented_objects_topic_ = nullptr;
  person_reid_topic_ = nullptr;
  person_attribs_topic_ = nullptr;
  face_reid_topic_ = nullptr;
  landmarks_topic_ = nullptr;
}

void Outputs::RosTopicOutput::feedFrame(const cv::Mat & frame)
{
  frame_ = frame.clone();
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::FaceReidentificationResult> & results)
{
  face_reid_topic_ = std::make_shared<people_msgs::msg::ReidentificationStamped>();
  people_msgs::msg::Reidentification face;
  for (auto & r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    face.roi.x_offset = loc.x;
    face.roi.y_offset = loc.y;
    face.roi.width = loc.width;
    face.roi.height = loc.height;
    face.identity = r.getFaceID();
    face_reid_topic_->reidentified_vector.push_back(face);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::LandmarksDetectionResult> & results)
{
  landmarks_topic_ = std::make_shared<people_msgs::msg::LandmarkStamped>();
  people_msgs::msg::Landmark landmark;
  for (auto & r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    landmark.roi.x_offset = loc.x;
    landmark.roi.y_offset = loc.y;
    landmark.roi.width = loc.width;
    landmark.roi.height = loc.height;
    std::vector<cv::Point> landmark_points = r.getLandmarks();
    for (auto pt : landmark_points) {
      geometry_msgs::msg::Point point;
      point.x = pt.x;
      point.y = pt.y;
      landmark.landmark_points.push_back(point);
    }
    landmarks_topic_->landmarks.push_back(landmark);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::PersonAttribsDetectionResult> & results)
{
  person_attribs_topic_ = std::make_shared<people_msgs::msg::PersonAttributeStamped>();
  people_msgs::msg::PersonAttribute person_attrib;
  for (auto & r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    person_attrib.roi.x_offset = loc.x;
    person_attrib.roi.y_offset = loc.y;
    person_attrib.roi.width = loc.width;
    person_attrib.roi.height = loc.height;
    person_attrib.attribute = r.getAttributes();
    person_attribs_topic_->attributes.push_back(person_attrib);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::PersonReidentificationResult> & results)
{
  person_reid_topic_ = std::make_shared<people_msgs::msg::ReidentificationStamped>();
  people_msgs::msg::Reidentification person;
  for (auto & r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    person.roi.x_offset = loc.x;
    person.roi.y_offset = loc.y;
    person.roi.width = loc.width;
    person.roi.height = loc.height;
    person.identity = r.getPersonID();
    person_reid_topic_->reidentified_vector.push_back(person);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectSegmentationResult> & results)
{
  segmented_objects_topic_ = std::make_shared<people_msgs::msg::ObjectsInMasks>();
  people_msgs::msg::ObjectInMask object;
  for (auto & r : results) {
    // slog::info << ">";
    auto loc = r.getLocation();
    object.roi.x_offset = loc.x;
    object.roi.y_offset = loc.y;
    object.roi.width = loc.width;
    object.roi.height = loc.height;
    object.object_name = r.getLabel();
    object.probability = r.getConfidence();
    cv::Mat mask = r.getMask();
    for (int h = 0; h < mask.size().height; ++h) {
      for (int w = 0; w < mask.size().width; ++w) {
        object.mask_array.push_back(mask.at<float>(h, w));
      }
    }
    segmented_objects_topic_->objects_vector.push_back(object);
  }
}

void Outputs::RosTopicOutput::accept(
  const std::vector<dynamic_vino_lib::ObjectDetectionResult> & results)
{
  detected_objects_topic_ = std::make_shared<object_msgs::msg::ObjectsInBoxes>();
  object_msgs::msg::ObjectInBox object;
  for (auto & r : results) {
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
  const std::vector<dynamic_vino_lib::FaceDetectionResult> & results)
{
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

void Outputs::RosTopicOutput::accept(const std::vector<dynamic_vino_lib::EmotionsResult> & results)
{
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

void Outputs::RosTopicOutput::accept(const std::vector<dynamic_vino_lib::AgeGenderResult> & results)
{
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
    if (male_prob > 0.5) {
      ag.gender = "Male";
      ag.gender_confidence = male_prob;
    } else {
      ag.gender = "Female";
      ag.gender_confidence = 1.0 - male_prob;
    }
    age_gender_topic_->objects.push_back(ag);
  }
}

void Outputs::RosTopicOutput::accept(const std::vector<dynamic_vino_lib::HeadPoseResult> & results)
{
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

void Outputs::RosTopicOutput::handleOutput()
{
  auto header = getHeader();
  if (landmarks_topic_ != nullptr) {
    // slog::info << "publishing landmarks detection outputs." << slog::endl;
    landmarks_topic_->header = header;
    pub_landmarks_->publish(landmarks_topic_);
    landmarks_topic_ = nullptr;
  }
  if (face_reid_topic_ != nullptr) {
    // slog::info << "publishing face reidentification outputs." << slog::endl;
    face_reid_topic_->header = header;
    pub_face_reid_->publish(face_reid_topic_);
    face_reid_topic_ = nullptr;
  }
  if (person_attribs_topic_ != nullptr) {
    // slog::info << "publishing person attributes outputs." << slog::endl;
    person_attribs_topic_->header = header;
    pub_person_attribs_->publish(person_attribs_topic_);
    person_attribs_topic_ = nullptr;
  }
  if (person_reid_topic_ != nullptr) {
    // slog::info << "publishing preson reidentification outputs." << slog::endl;
    person_reid_topic_->header = header;
    pub_person_reid_->publish(person_reid_topic_);
    person_reid_topic_ = nullptr;
  }
  if (segmented_objects_topic_ != nullptr) {
    // slog::info << "publishing segmented objects outputs." << slog::endl;
    segmented_objects_topic_->header = header;
    pub_segmented_object_->publish(segmented_objects_topic_);
    segmented_objects_topic_ = nullptr;
  }
  if (detected_objects_topic_ != nullptr) {
    // slog::info << "publishing detected objects outputs." << slog::endl;
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
std_msgs::msg::Header Outputs::RosTopicOutput::getHeader()
{
  return getPipeline()->getInputDevice()->getHeader();
}
