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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <ament_index_cpp/get_resource.hpp>
#include <openvino_param_lib/param_manager.hpp>

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

#include "openvino_wrapper_lib/pipeline.hpp"
#include "openvino_wrapper_lib/pipeline_manager.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino/openvino.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

static bool test_pass = false;

template<typename DurationT>
void wait_for_future(
  rclcpp::Executor & executor, std::shared_future<bool> & future,
  const DurationT & timeout)
{
  using rclcpp::FutureReturnCode;
  rclcpp::FutureReturnCode future_ret;
  auto start_time = std::chrono::steady_clock::now();
  future_ret = executor.spin_until_future_complete(future, timeout);
  auto elapsed_time = std::chrono::steady_clock::now() - start_time;
  EXPECT_EQ(FutureReturnCode::SUCCESS, future_ret) <<
    "the usb camera don't publish data to topic\n" <<
    "future failed to be set after: " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
    " milliseconds\n";
}

TEST(UnitTestObjectDetection, testObjectDetection)
{
  auto node = rclcpp::Node::make_shared("openvino_objectDetection_test");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  std::promise<bool> sub_called;
  std::shared_future<bool> sub_called_future(sub_called.get_future());

  auto openvino_faceDetection_callback =
    [&sub_called](const object_msgs::msg::ObjectsInBoxes::SharedPtr msg) -> void {
      test_pass = true;
      sub_called.set_value(true);
    };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto sub1 = node->create_subscription<object_msgs::msg::ObjectsInBoxes>(
      "/ros2_openvino_toolkit/detected_objects", qos, openvino_faceDetection_callback);

    executor.spin_once(std::chrono::seconds(0));

    wait_for_future(executor, sub_called_future, std::chrono::seconds(20));

    EXPECT_TRUE(test_pass);
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto offset = std::chrono::seconds(60);
  system("ros2 launch openvino_test pipeline_object_test.launch.py &");
  int ret = RUN_ALL_TESTS();
  rclcpp::sleep_for(offset);
  system("killall -s SIGINT pipeline_with_params &");
  rclcpp::shutdown();
  return ret;
}
