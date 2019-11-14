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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <people_msgs/msg/emotion.hpp>
#include <people_msgs/msg/emotions_stamped.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <people_msgs/msg/head_pose_stamped.hpp>
#include <people_msgs/msg/age_gender_stamped.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <vino_param_lib/param_manager.hpp>

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/pipeline_manager.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "extension/ext_list.hpp"
#include "inference_engine.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
static bool face_test_pass = false;
static bool emotion_test_pass = false;
static bool ageGender_test_pass = false;
static bool headPose_test_pass = false;

template<typename DurationT>
void wait_for_future(
  rclcpp::executor::Executor & executor, std::shared_future<bool> & future,
  const DurationT & timeout)
{
  using rclcpp::executor::FutureReturnCode;
  rclcpp::executor::FutureReturnCode future_ret;
  auto start_time = std::chrono::steady_clock::now();
  future_ret = executor.spin_until_future_complete(future, timeout);
  auto elapsed_time = std::chrono::steady_clock::now() - start_time;
  EXPECT_EQ(FutureReturnCode::SUCCESS, future_ret) <<
    "the usb camera don't publish data to topic\n" <<
    "future failed to be set after: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
    " milliseconds\n";
}

TEST(UnitTestFaceDetection, testFaceDetection)
{
  auto node = rclcpp::Node::make_shared("openvino_face_test");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  std::promise<bool> sub_called;
  std::shared_future<bool> sub_called_future(sub_called.get_future());

  auto openvino_faceDetection_callback =
    [&sub_called](const object_msgs::msg::ObjectsInBoxes::SharedPtr msg) -> void {
      face_test_pass = true;
      sub_called.set_value(true);
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto sub1 = node->create_subscription<object_msgs::msg::ObjectsInBoxes>(
      "/ros2_openvino_toolkit/face_detection", qos, openvino_faceDetection_callback);

    executor.spin_once(std::chrono::seconds(0));

    wait_for_future(executor, sub_called_future, std::chrono::seconds(10));

    EXPECT_TRUE(face_test_pass);
  }
}

TEST(UnitTestFaceDetection, testEmotionDetection)
{
  auto node = rclcpp::Node::make_shared("openvino_emotion_test");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  std::promise<bool> sub_called;
  std::shared_future<bool> sub_called_future(sub_called.get_future());

  auto openvino_emotionRecognition_callback =
    [&sub_called](const people_msgs::msg::EmotionsStamped::SharedPtr msg) -> void {
      emotion_test_pass = true;
      sub_called.set_value(true);
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto sub2 = node->create_subscription<people_msgs::msg::EmotionsStamped>(
      "/ros2_openvino_toolkit/emotions_recognition", qos, openvino_emotionRecognition_callback);

    executor.spin_once(std::chrono::seconds(0));

    wait_for_future(executor, sub_called_future, std::chrono::seconds(10));

    EXPECT_TRUE(emotion_test_pass);
  }
}

TEST(UnitTestFaceDetection, testageGenderDetection)
{
  auto node = rclcpp::Node::make_shared("openvino_ageGender_test");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  std::promise<bool> sub_called;
  std::shared_future<bool> sub_called_future(sub_called.get_future());

  auto openvino_ageGender_callback =
    [&sub_called](const people_msgs::msg::AgeGenderStamped::SharedPtr msg) -> void {
      ageGender_test_pass = true;
      sub_called.set_value(true);
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto sub3 = node->create_subscription<people_msgs::msg::AgeGenderStamped>(
      "/ros2_openvino_toolkit/age_genders_Recognition", qos, openvino_ageGender_callback);

    executor.spin_once(std::chrono::seconds(0));

    wait_for_future(executor, sub_called_future, std::chrono::seconds(10));

    EXPECT_TRUE(ageGender_test_pass);
  }
}

TEST(UnitTestFaceDetection, testheadPoseDetection)
{
  auto node = rclcpp::Node::make_shared("openvino_headPose_test");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  std::promise<bool> sub_called;
  std::shared_future<bool> sub_called_future(sub_called.get_future());

  auto openvino_headPose_callback =
    [&sub_called](const people_msgs::msg::HeadPoseStamped::SharedPtr msg) -> void {
      headPose_test_pass = true;
      sub_called.set_value(true);
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto sub4 = node->create_subscription<people_msgs::msg::HeadPoseStamped>(
      "/ros2_openvino_toolkit/headposes_estimation", qos, openvino_headPose_callback);

    executor.spin_once(std::chrono::seconds(0));

    wait_for_future(executor, sub_called_future, std::chrono::seconds(10));

    EXPECT_TRUE(headPose_test_pass);
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto offset = std::chrono::seconds(30);
  system("ros2 launch dynamic_vino_sample pipeline_face_test.launch.py &");
  int ret = RUN_ALL_TESTS();
  rclcpp::sleep_for(offset);
  system("killall -s SIGINT pipeline_with_params &");
  rclcpp::shutdown();
  return ret;
}
