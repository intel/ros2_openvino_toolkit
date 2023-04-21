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
* \brief A sample for this library. This sample performs face detection,
 * emotions detection, age gender detection and head pose estimation.
* \file sample/pipeline_manager.cpp
*/

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <openvino_param_lib/param_manager.hpp>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <csignal>
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
#include "openvino_wrapper_lib/services/pipeline_processing_server.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#if(defined(USE_OLD_E_PLUGIN_API))
#include <extension/ext_list.hpp>
#endif
#include "openvino/openvino.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "utility.hpp"

void signalHandler(int signum)
{
  slog::warn << "!!!!!!!!!!!Interrupt signal (" << signum << ") received!!!!!!!!!!!!" << slog::endl;

  // cleanup and close up stuff here
  // terminate program
  PipelineManager::getInstance().stopAll();
  // exit(signum);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Node::SharedPtr main_node = rclcpp::Node::make_shared("openvino_pipeline");
  rclcpp::Node::SharedPtr service_node = std::make_shared<vino_service::PipelineProcessingServer
      <openvino_msgs::srv::PipelineSrv>>("pipeline_service");
  // register signal SIGINT and signal handler
  //signal(SIGINT, signalHandler);

  try {
    std::cout << "OpenVINO: " << ov::get_openvino_version() << std::endl;

    // ----- Parsing and validation of input args-----------------------
    std::string config = getConfigPath(argc, argv);
    if(config.empty()){
      throw std::runtime_error("Config File is not correctly set.");
      return -1;
    }
    slog::info << "Config File Path =" << config << slog::endl;

    Params::ParamManager::getInstance().parse(config);
    Params::ParamManager::getInstance().print();
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    if (pipelines.size() < 1) {
      throw std::logic_error("Pipeline parameters should be set!");
    }
    for (auto & p : pipelines) {
      PipelineManager::getInstance().createPipeline(p, main_node);
    }

    PipelineManager::getInstance().runAll();

    //rclcpp::spin(main_node);
    exec.add_node(main_node);
    exec.add_node(service_node);
    exec.spin();
    PipelineManager::getInstance().stopAll();
    rclcpp::shutdown();

  } catch (const std::exception & error) {
    slog::err << error.what() << slog::endl;
    return -2;
  } catch (...) {
    slog::err << "Unknown/internal exception happened." << slog::endl;
    return -3;
  }

  return 0;
}
