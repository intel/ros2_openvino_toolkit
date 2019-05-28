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
 * @brief a header file with declaration of RealSenseCamera class
 * @file realsense_camera_topic.cpp
 */

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include "dynamic_vino_lib/inputs/realsense_camera_topic.hpp"
#include "dynamic_vino_lib/slog.hpp"

#define INPUT_TOPIC "/openvino_toolkit/image_raw"

Input::RealSenseCameraTopic::RealSenseCameraTopic()
: Node("realsense_topic")
{
}

bool Input::RealSenseCameraTopic::initialize()
{
  slog::info << "before cameraTOpic init" << slog::endl;
  std::shared_ptr<rclcpp::Node> node(this);
  setHandler(node);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
  sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/openvino_toolkit/image_raw", qos,
    std::bind(&RealSenseCameraTopic::cb, this, std::placeholders::_1));

  image_count_ = 0;
  return true;
}

void Input::RealSenseCameraTopic::cb(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  slog::info << "Receiving a new image from Camera topic." << slog::endl;
  setHeader(image_msg->header);
  image_ = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  ++image_count_;
}

bool Input::RealSenseCameraTopic::read(cv::Mat * frame)
{
  if (image_.empty() || image_count_ <= 0) {
    slog::warn << "No data received in CameraTopic instance" << slog::endl;
    return false;
  }
  *frame = image_;
  --image_count_;
  return true;
}
