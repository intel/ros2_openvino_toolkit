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
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "dynamic_vino_lib/services/frame_processing_server.hpp"
#include "dynamic_vino_lib/pipeline_manager.hpp"
#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/inputs/base_input.hpp"
#include "dynamic_vino_lib/inputs/image_input.hpp"
#include "inference_engine.hpp"
#include "extension/ext_list.hpp"

std::string getConfigPath(int argc, char * argv[])
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "dynamic_vino_sample", content, &prefix_path);
  return prefix_path + "/share/dynamic_vino_sample/param/image_object_server.yaml";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string config_path = getConfigPath(argc, argv);
  std::cout << "***config path is " << config_path << std::endl;

  try {
    std::string service_name = "frame_processing_server";
    auto node = std::make_shared<vino_service::FrameProcessingServer
        <object_msgs::srv::DetectObject>>(service_name, config_path);
    rclcpp::spin(node);
  } catch (...) {
    std::cout << "[ERROR] [frame_processing_server]: " <<
      "exception caught" << std::endl;
  }
}
