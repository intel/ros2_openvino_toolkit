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

#include <ament_index_cpp/get_resource.hpp>
#include <vino_param_lib/param_manager.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <iomanip>

#include "dynamic_vino_lib/services/frame_processing_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("service_example_for_face");
  if (argc != 2) {
    RCLCPP_INFO(node->get_logger(), "Usage: ros2 run dynamic_vino_sample image_object_client"
      "<image_path>");
    return -1;
  }

  std::string image_path = argv[1];

  auto client_face = node->create_client<object_msgs::srv::DetectObject>("/detect_face");
  auto request_face = std::make_shared<object_msgs::srv::DetectObject::Request>();
  request_face->image_path = image_path;

  auto client_age_gender = node->create_client<people_msgs::srv::AgeGender>("/detect_age_gender");
  auto request_age_gender = std::make_shared<people_msgs::srv::AgeGender::Request>();
  request_age_gender->image_path = image_path;

  auto client_emotion = node->create_client<people_msgs::srv::Emotion>("/detect_emotion");
  auto request_emotion = std::make_shared<people_msgs::srv::Emotion::Request>();
  request_emotion->image_path = image_path;

  auto client_headpose = node->create_client<people_msgs::srv::HeadPose>("/detect_head_pose");
  auto request_headpose = std::make_shared<people_msgs::srv::HeadPose::Request>();
  request_headpose->image_path = image_path;

  while (!client_face->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result_face = client_face->async_send_request(request_face);
  auto result_age_gender = client_age_gender->async_send_request(request_age_gender);
  auto result_emotion = client_emotion->async_send_request(request_emotion);
  auto result_headpose = client_headpose->async_send_request(request_headpose);

  if (rclcpp::spin_until_future_complete(node, result_face) ==
    rclcpp::executor::FutureReturnCode::SUCCESS && rclcpp::spin_until_future_complete(node,
    result_age_gender) == rclcpp::executor::FutureReturnCode::SUCCESS &&
    rclcpp::spin_until_future_complete(node, result_emotion) ==
    rclcpp::executor::FutureReturnCode::SUCCESS && rclcpp::spin_until_future_complete(node,
    result_headpose) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto srv_face = result_face.get();
    auto srv_age_gender = result_age_gender.get();
    auto srv_emotion = result_emotion.get();
    auto srv_headpose = result_headpose.get();

    cv::Mat image = cv::imread(image_path);
    int width = image.cols;
    int height = image.rows;

    for (unsigned int i = 0; i < srv_face->objects.objects_vector.size(); i++) {
      std::stringstream ss;
      ss << "[" << srv_face->objects.objects_vector[i].object.object_name << "]";
      ss << "[" << srv_face->objects.objects_vector[i].object.probability * 100 << "%" << "]";
      ss << "[" << srv_emotion->emotion.emotions[i].emotion << "]";
      ss << "[Y" << std::fixed << std::setprecision(0) <<
        srv_age_gender->age_gender.objects[i].age << "]";
      RCLCPP_INFO(node->get_logger(), "%d: object: %s", i,
        srv_face->objects.objects_vector[i].object.object_name.c_str());
      RCLCPP_INFO(node->get_logger(), "prob: %f",
        srv_face->objects.objects_vector[i].object.probability);
      RCLCPP_INFO(
        node->get_logger(), "location: (%d, %d, %d, %d)",
        srv_face->objects.objects_vector[i].roi.x_offset,
        srv_face->objects.objects_vector[i].roi.y_offset,
        srv_face->objects.objects_vector[i].roi.width,
        srv_face->objects.objects_vector[i].roi.height);
      RCLCPP_INFO(node->get_logger(), "Yaw, Pitch and Roll for head pose is: (%f, %f, %f),",
        srv_headpose->headpose.headposes[i].yaw, srv_headpose->headpose.headposes[i].pitch,
        srv_headpose->headpose.headposes[i].roll);

      int xmin = srv_face->objects.objects_vector[i].roi.x_offset;
      int ymin = srv_face->objects.objects_vector[i].roi.y_offset;
      int w = srv_face->objects.objects_vector[i].roi.width;
      int h = srv_face->objects.objects_vector[i].roi.height;

      int xmax = ((xmin + w) < width) ? (xmin + w) : width;
      int ymax = ((ymin + h) < height) ? (ymin + h) : height;

      cv::Point left_top = cv::Point(xmin, ymin);
      cv::Point right_bottom = cv::Point(xmax, ymax);
      if (srv_age_gender->age_gender.objects[i].gender == "Male") {
        cv::rectangle(image, left_top, right_bottom, cv::Scalar(255, 0, 0), 1, 1, 0);
        cv::rectangle(image, cvPoint(xmin, ymin), cvPoint(xmax, ymin), cv::Scalar(255, 0, 0),
          -1);
        cv::putText(image, ss.str(), cvPoint(xmin, ymin - 10), cv::FONT_HERSHEY_PLAIN, 1,
          cv::Scalar(255, 0, 0), 1);
      } else {
        cv::rectangle(image, left_top, right_bottom, cv::Scalar(0, 0, 255), 1, 1, 0);
        cv::rectangle(image, cvPoint(xmin, ymin), cvPoint(xmax, ymin), cv::Scalar(0, 0, 255),
          -1);
        cv::putText(image, ss.str(), cvPoint(xmin, ymin - 10), cv::FONT_HERSHEY_PLAIN, 1,
          cv::Scalar(0, 0, 255), 1);
      }
    }
    cv::imshow("image_detection", image);
    cv::waitKey(0);
  }
}
