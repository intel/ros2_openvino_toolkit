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
 * @brief a header file with declaration of ImageTopic class
 * @file image_topic.cpp
 */

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include "openvino_wrapper_lib/inputs/image_topic.hpp"
#include "openvino_wrapper_lib/slog.hpp"

#define INPUT_TOPIC "/openvino_toolkit/image_raw"


Input::ImageTopic::ImageTopic(rclcpp::Node::SharedPtr node)
: node_(node)
{
}

bool Input::ImageTopic::initialize()
{
  slog::debug << "before Image Topic init" << slog::endl;

  if(node_ == nullptr){
    throw std::runtime_error("Image Topic is not instancialized because of no parent node.");
    return false;
  }
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    INPUT_TOPIC, qos,
    std::bind(&ImageTopic::cb, this, std::placeholders::_1));

  return true;
}

  bool Input::ImageTopic::initialize(size_t width, size_t height)
  {
    slog::warn << "BE CAREFUL: nothing for resolution is done when calling initialize(width, height)"
      << " for Image Topic" << slog::endl;
    return initialize();
  }

void Input::ImageTopic::cb(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
  slog::debug << "Receiving a new image from Camera topic." << slog::endl;
  setHeader(image_msg->header);

  image_ = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
  image_count_.increaseCounter();
}

bool Input::ImageTopic::read(cv::Mat * frame)
{
  if (image_count_.get() < 0 || image_.empty()) {
    slog::debug << "No data received in CameraTopic instance" << slog::endl;
    return false;
  }

  *frame = image_;
  lockHeader();
  image_count_.decreaseCounter();
  return true;
}
