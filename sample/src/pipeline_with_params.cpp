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
* \brief A sample for this library. This sample performs face detection,
 * emotions detection, age gender detection and head pose estimation.
* \file sample/main.cpp
*/

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
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_resource.hpp>
#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/pipeline_manager.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "extension/ext_list.hpp"
#include "gflags/gflags.h"
#include "inference_engine.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "utility.hpp"

using namespace InferenceEngine;
using namespace rs2;

void signalHandler(int signum) {
  slog::warn << "!!!!!!!!!!!Interrupt signal (" << signum
             << ") received!!!!!!!!!!!!" << slog::endl;

  // cleanup and close up stuff here
  // terminate program
  PipelineManager::getInstance().stopAll();
  // exit(signum);
}

bool parseAndCheckCommandLine(int argc, char** argv) {
  // -----Parsing and validation of input args---------------------------
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (FLAGS_h) {
    showUsageForParam();
    return false;
  }

  return true;
}

std::string getConfigPath(int argc, char* argv[]) {
  if (parseAndCheckCommandLine(argc, argv)) {
    if (!FLAGS_config.empty()) {
      return FLAGS_config;
    }
  }

  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "dynamic_vino_sample", content,
                                &prefix_path);
  // slog::info << "prefix_path=" << prefix_path << slog::endl;
  return prefix_path + "/share/dynamic_vino_sample/param/pipeline_people.yaml";
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  try {
    std::cout << "InferenceEngine: " << GetInferenceEngineVersion()
              << std::endl;

    // ----- Parsing and validation of input args-----------------------

    std::string config = getConfigPath(argc, argv);
    slog::info << "Config File Path =" << config << slog::endl;

    Params::ParamManager::getInstance().parse(config);
    Params::ParamManager::getInstance().print();
    auto pcommon = Params::ParamManager::getInstance().getCommon();
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    if (pipelines.size() < 1) {
      throw std::logic_error("Pipeline parameters should be set!");
    }
    // auto createPipeline = PipelineManager::getInstance().createPipeline;
    for (auto& p : pipelines) {
      PipelineManager::getInstance().createPipeline(p);
    }

    PipelineManager::getInstance().runAll();
    PipelineManager::getInstance().joinAll();

  } catch (const std::exception& error) {
    slog::err << error.what() << slog::endl;
    return 1;
  } catch (...) {
    slog::err << "Unknown/internal exception happened." << slog::endl;
    return 1;
  }
}
