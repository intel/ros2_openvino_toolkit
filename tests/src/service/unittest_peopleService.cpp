// Copyright (c) 2017-2022 Intel Corporation. All Rights Reserved
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
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "openvino_wrapper_lib/services/frame_processing_server.hpp"

std::string generate_file_path(std::string path)
{
  std::string base_path = __FILE__;
  const std::string filename = "tests/src/service/unittest_peopleService.cpp";
  base_path = base_path.substr(0, base_path.length() - filename.length() - 1);
  return base_path + "/" + path;
}

TEST(UnitTestPeople, testPeople)
{
  auto node = rclcpp::Node::make_shared("openvino_people_service_test");

  auto client = node->create_client<object_msgs::srv::People>("/openvino_toolkit/service");
  
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(20)));
  
  auto request = std::make_shared<object_msgs::srv::People::Request>();

  std::string buffer = generate_file_path("data/images/team.jpg");
  std::cout << buffer << std::endl;
  request->image_path = buffer;

  auto result = client->async_send_request(request);

  ASSERT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    rclcpp::spin_until_future_complete(node, result));

  auto srv = result.get();

  EXPECT_TRUE(srv->persons.faces.size());

  for (unsigned int i = 0; i < srv->persons.faces.size(); i++) {
    EXPECT_EQ(srv->persons.faces[i].object.object_name, "Person");
    EXPECT_TRUE(srv->persons.emotions[i].emotion.c_str());
    EXPECT_TRUE(srv->persons.agegenders[i].age);
    EXPECT_TRUE(srv->persons.agegenders[i].gender.c_str());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  auto offset = std::chrono::seconds(20);
  system("ros2 launch openvino_test image_people_service_test.launch.py &");
  rclcpp::sleep_for(offset);
  int ret = RUN_ALL_TESTS();
  system("killall -s SIGINT image_people_server &");
  rclcpp::shutdown();
  return ret;
}
