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

#include <object_msgs/srv/people.hpp>
#include <object_msgs/msg/persons_stamped.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <openvino_param_lib/param_manager.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <iomanip>

#include "openvino_wrapper_lib/services/frame_processing_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("service_example_for_face");
  if (argc != 2) {
    RCLCPP_INFO(node->get_logger(), "Usage: ros2 run openvino_node image_object_client"
      "<image_path>");
    return -1;
  }

  std::string image_path = argv[1];

  auto client = node->create_client<object_msgs::srv::People>("/openvino_toolkit/service");
  auto request = std::make_shared<object_msgs::srv::People::Request>();
  request->image_path = image_path;

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto people = result.get();
    if (people->persons.emotions.size() == 0 && people->persons.agegenders.size() == 0 &&
      people->persons.headposes.size() == 0)
    {
      RCLCPP_INFO(node->get_logger(), "Get response, but no any person found.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Found persons...");

    for (unsigned int i = 0; i < people->persons.faces.size(); i++) {
      RCLCPP_INFO(node->get_logger(), "%d: object: %s", i,
        people->persons.faces[i].object.object_name.c_str());
      RCLCPP_INFO(node->get_logger(), "prob: %f",
        people->persons.faces[i].object.probability);
      RCLCPP_INFO(
        node->get_logger(), "location: (%d, %d, %d, %d)",
        people->persons.faces[i].roi.x_offset, people->persons.faces[i].roi.y_offset,
        people->persons.faces[i].roi.width, people->persons.faces[i].roi.height);
      RCLCPP_INFO(node->get_logger(), "Emotions: %s",
        people->persons.emotions[i].emotion.c_str());
      RCLCPP_INFO(node->get_logger(), "Age: %f, Gender: %s",
        people->persons.agegenders[i].age, people->persons.agegenders[i].gender.c_str());
      RCLCPP_INFO(node->get_logger(), "Yaw, Pitch and Roll for head pose is: (%f, %f, %f),",
        people->persons.headposes[i].yaw, people->persons.headposes[i].pitch,
        people->persons.headposes[i].roll);
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "NO response received!!");
  }
}
