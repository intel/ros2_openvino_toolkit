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
* \brief The is the composition version making the pipeline management into
* a rclcpp Node.
* \file pipeline_composite.cpp
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
#include "openvino_wrapper_lib/slog.hpp"
#if(defined(USE_OLD_E_PLUGIN_API))
#include <extension/ext_list.hpp>
#endif
#include "openvino/openvino.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
//#include "utility.hpp"

void signalHandler(int signum)
{
  slog::warn << "!!!!!!!!!!!Interrupt signal (" << signum << ") received!!!!!!!!!!!!" << slog::endl;

  // cleanup and close up stuff here
  // terminate program
  PipelineManager::getInstance().stopAll();
  // exit(signum);
}

class ComposablePipeline : public rclcpp::Node
{
public:
  ComposablePipeline(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions())
  : rclcpp::Node("composable_pipeline", "/", node_options)
  {
    initPipeline();
  }
  virtual ~ComposablePipeline() = default;

private:
  void initPipeline()
  {
    // register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    std::string config = getConfigPath();
    slog::info << "Config File Path =" << config << slog::endl;

    Params::ParamManager::getInstance().parse(config);
    Params::ParamManager::getInstance().print();
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    if (pipelines.size() < 1) {
      throw std::logic_error("Pipeline parameters should be set!");
    }

    std::shared_ptr<rclcpp::Node> node_handler(this);
    for (auto & p : pipelines) {
      PipelineManager::getInstance().createPipeline(p, node_handler);
    }

    PipelineManager::getInstance().runAll();
  }

  std::string getConfigPath()
  {
    // TODO: Fix api for humble
    // return declare_parameter("config").get<rclcpp::PARAMETER_STRING>();
    return "";
  }

};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ComposablePipeline)
