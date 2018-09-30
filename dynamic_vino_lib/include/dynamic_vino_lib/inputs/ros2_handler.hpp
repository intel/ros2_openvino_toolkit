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
 * @brief A header file with declaration for ROS/ROS2 handler
 * @file ros_handler.h
 */

#ifndef DYNAMIC_VINO_LIB__INPUTS__ROS2_HANDLER_HPP_
#define DYNAMIC_VINO_LIB__INPUTS__ROS2_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace Input {

class Ros2Handler {
public:
  void setHandler(const std::shared_ptr<rclcpp::Node>& node) {
    node_ = node;
  }
  std::shared_ptr<rclcpp::Node> getHandler() const{
    return node_;
  }
private:
  std::shared_ptr<rclcpp::Node> node_;
  
};
  
} // namespace


#endif // DYNAMIC_VINO_LIB__INPUTS__ROS_HANDLER_HPP_