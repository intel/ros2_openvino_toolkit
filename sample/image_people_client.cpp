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

#include "dynamic_vino_lib/services/frame_processing_server.hpp"
#include <ament_index_cpp/get_resource.hpp>
#include <vino_param_lib/param_manager.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  std::cout << "object client" << std::endl;
  auto node = rclcpp::Node::make_shared("service_example");
 
  std::string image_path = "/home/intel/Pictures/timg.jpeg";

  std::cout << "******0000" << std::endl;

  auto client =
    node->create_client<object_msgs::srv::DetectObject>("/detect_face");
  std::cout << "******face client created" << std::endl;
  auto request = std::make_shared<object_msgs::srv::DetectObject::Request>();
  std::cout << "******face request created" << std::endl;

  auto client =
    node->create_client<object_msgs::srv::DetectObject>("/detect_face");
  std::cout << "******face client created" << std::endl;
  auto request = std::make_shared<object_msgs::srv::DetectObject::Request>();
  std::cout << "******face request created" << std::endl;

  //request->image_path = argv[1];
  request->image_path = image_path;

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  
  std::cout << "******result got" << std::endl;

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto srv = result.get();


  std::cout << "******srv got" << std::endl;

  std::cout << "***********" << srv->objects.inference_time_ms << std::endl;
  }
}
