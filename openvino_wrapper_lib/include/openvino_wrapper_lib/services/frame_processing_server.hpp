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
#ifndef OPENVINO_WRAPPER_LIB__SERVICES__FRAME_PROCESSING_SERVER_HPP_
#define OPENVINO_WRAPPER_LIB__SERVICES__FRAME_PROCESSING_SERVER_HPP_

#include <object_msgs/msg/object.hpp>
#include <object_msgs/msg/object_in_box.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <object_msgs/msg/emotion.hpp>
#include <object_msgs/msg/emotions_stamped.hpp>
#include <object_msgs/msg/age_gender.hpp>
#include <object_msgs/msg/age_gender_stamped.hpp>
#include <object_msgs/msg/head_pose.hpp>
#include <object_msgs/msg/head_pose_stamped.hpp>

#include <object_msgs/srv/age_gender_srv.hpp>
#include <object_msgs/srv/emotion_srv.hpp>
#include <object_msgs/srv/head_pose_srv.hpp>
#include <object_msgs/srv/detect_object.hpp>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <string>

namespace vino_service
{
template<typename T>
class FrameProcessingServer : public rclcpp::Node
{
public:
  explicit FrameProcessingServer(
    const std::string & service_name,
    const std::string & config_path);
  void initService(const std::string & config_path);

private:
  void cbService(
    const std::shared_ptr<typename T::Request> request,
    std::shared_ptr<typename T::Response> response);

  // rclcpp::Service<T::template>::SharedPtr service_;
  std::shared_ptr<rclcpp::Service<T>> service_;
  std::string service_name_;
  std::string config_path_;
};
}  // namespace vino_service
#endif  // OPENVINO_WRAPPER_LIB__SERVICES__FRAME_PROCESSING_SERVER_HPP_
