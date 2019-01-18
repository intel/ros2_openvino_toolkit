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
#ifndef DYNAMIC_VINO_LIB__FRAME_PROCESSING_SERVER_HPP_
#define DYNAMIC_VINO_LIB__FRAME_PROCESSING_SERVER_HPP_

#include <memory>
#include <iostream>
#include <object_msgs/msg/object.hpp>                                                                 
#include <object_msgs/msg/object_in_box.hpp>                                                          
#include <object_msgs/msg/objects_in_boxes.hpp>                                                       
#include <people_msgs/msg/emotion.hpp>                                                                
#include <people_msgs/msg/emotions_stamped.hpp>                                                       
#include <people_msgs/msg/age_gender.hpp>                                                             
#include <people_msgs/msg/age_gender_stamped.hpp>                                                     
#include <people_msgs/msg/head_pose.hpp>                                                              
#include <people_msgs/msg/head_pose_stamped.hpp>

#include <people_msgs/srv/age_gender.hpp>
#include <people_msgs/srv/emotion.hpp>
#include <people_msgs/srv/head_pose.hpp>
#include <object_msgs/srv/detect_object.hpp>

#include <rclcpp/rclcpp.hpp>

namespace vino_service {

class FrameProcessingServer : public rclcpp::Node {
public:
  explicit FrameProcessingServer(const std::string service_name, const std::string config_path);

private:
  void cbFaceDetection(const std::shared_ptr<object_msgs::srv::DetectObject::Request> request, 
          std::shared_ptr<object_msgs::srv::DetectObject::Response> response);

  void cbAgeGenderRecognition(const std::shared_ptr<people_msgs::srv::AgeGender::Request> request, 
          std::shared_ptr<people_msgs::srv::AgeGender::Response> response);

  void cbEmotionRecognition(const std::shared_ptr<people_msgs::srv::Emotion::Request> request, 
          std::shared_ptr<people_msgs::srv::Emotion::Response> response);

  void cbHeadPoseRecognition(const std::shared_ptr<people_msgs::srv::HeadPose::Request> request, 
          std::shared_ptr<people_msgs::srv::HeadPose::Response> response);

  void cbObjectDetection(const std::shared_ptr<object_msgs::srv::DetectObject::Request> request, 
          std::shared_ptr<object_msgs::srv::DetectObject::Response> response);

  rclcpp::Service<object_msgs::srv::DetectObject>::SharedPtr face_service_;
  rclcpp::Service<people_msgs::srv::AgeGender>::SharedPtr age_gender_service_;
  rclcpp::Service<people_msgs::srv::Emotion>::SharedPtr emotion_service_;
  rclcpp::Service<people_msgs::srv::HeadPose>::SharedPtr head_pose_service_;
  rclcpp::Service<object_msgs::srv::DetectObject>::SharedPtr object_service_;
};
}  //namespace frame_processing_service
#endif  // DYNAMIC_VINO_LIB__FRAME_PROCESSING_SERVER_HPP_
