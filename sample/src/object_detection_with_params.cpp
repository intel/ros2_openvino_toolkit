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
#include <rclcpp/rclcpp.hpp>
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

#include <ament_index_cpp/get_resource.hpp>
#include "dynamic_vino_lib/common.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/factory.hpp"
#include "dynamic_vino_lib/inferences/age_gender_detection.hpp"
#include "dynamic_vino_lib/inferences/base_inference.hpp"
#include "dynamic_vino_lib/inferences/emotions_detection.hpp"
#include "dynamic_vino_lib/inferences/face_detection.hpp"
#include "dynamic_vino_lib/inferences/head_pose_detection.hpp"
#include "dynamic_vino_lib/inputs/realsense_camera_topic.hpp"
#include "dynamic_vino_lib/outputs/image_window_output.hpp"
#include "dynamic_vino_lib/outputs/ros_topic_output.hpp"
#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "extension/ext_list.hpp"
#include "gflags/gflags.h"
#include "inference_engine.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "utility.hpp"
using namespace InferenceEngine;
using namespace rs2;
bool parseAndCheckCommandLine(int argc, char** argv) {
  // ---------------------------Parsing and validation of input
  // args-----------------------------
  gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
  if (FLAGS_h) {
    showUsageForObjectDetection();
    return false;
  }
/*  return true;
  slog::info << "Parsing input parameters" << slog::endl;
  if (FLAGS_i.empty()) {
    throw std::logic_error("Parameter -i is not set");
  }
  
  if (FLAGS_m.empty()) {
    throw std::logic_error("Parameter -m is not set");
  }
  */
  return true;
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "dynamic_vino_sample", content,
                                &prefix_path);
  slog::info << "prefix_path=" << prefix_path << slog::endl;

  try {
    std::cout << "InferenceEngine: " << GetInferenceEngineVersion()
              << std::endl;
    // ------------------------------ Parsing and validation of input args
    // -------------------------
    if (!parseAndCheckCommandLine(argc, argv)) {
      return 0;
    }
    if (FLAGS_config.empty()) {
      FLAGS_config =
          prefix_path + "/share/dynamic_vino_sample/param/pipeline_object.yaml";
    }
    Params::ParamManager::getInstance().parse(FLAGS_config);
    Params::ParamManager::getInstance().print();
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    if (pipelines.size() < 1) {
      throw std::logic_error("Pipeline parameters should be set!");
    }

    FLAGS_i = pipelines[0].inputs[0];
    FLAGS_m = pipelines[0].infers[0].model;

    // ----------- 1. Load Plugin for inference engine
    std::unique_ptr<InferencePlugin> plugin = Factory::makePluginByName(
      FLAGS_d, FLAGS_l, FLAGS_c, FLAGS_pc);
    
    // --------------------------- 2. Generate Input Device and Output
    // Device-----------------------
    slog::info << "Reading input" << slog::endl;
    auto input_ptr = Factory::makeInputDeviceByName(FLAGS_i);
 
    // add node handler to input device instance
    // input_ptr->setHandler(node);
    if (!input_ptr->initialize()) {
      throw std::logic_error("Cannot open input file or camera: " + FLAGS_i);
    }
    std::string window_name = "Results";
    auto output_ptr = std::make_shared<Outputs::ImageWindowOutput>(window_name);
    // --------------------------- 3. Generate Inference
    // Instance-----------------------------------
    // generate face detection inference
    auto model =
        std::make_shared<Models::ObjectDetectionModel>(FLAGS_m, 1, 1, 1);
    model->modelInit();
    auto engine = std::make_shared<Engines::Engine>(*plugin, model);
    auto object_detection_ptr =
        std::make_shared<dynamic_vino_lib::ObjectDetection>(FLAGS_t);
    object_detection_ptr->loadNetwork(model);
    object_detection_ptr->loadEngine(engine);
    slog::info << "Pass Inference Prep." << slog::endl;
    // ------- 4. Build Pipeline -------------
    Pipeline pipe("object");
    auto pipeline_params = Params::ParamManager::getInstance().getPipeline("object");
    pipe.setParams(pipeline_params);
    pipe.add("video_input", input_ptr);
    pipe.add("video_input", "object_detection", object_detection_ptr);
    pipe.add("object_detection", "video_output", output_ptr);
    auto ros_topic_output_ptr = std::make_shared<Outputs::RosTopicOutput>();
    pipe.add("object_detection", "ros_output", ros_topic_output_ptr);
    pipe.setCallback();
    pipe.printPipeline();
    
    slog::info << "Pass Pipeline Init." << slog::endl;

    // ------- 5. Run Pipeline -----------
    auto node = input_ptr->getHandler();
    while (cv::waitKey(1) < 0 && cvGetWindowHandle(window_name.c_str())) {
      if (node != nullptr) {
        rclcpp::spin_some(node);
      }
      pipe.runOnce();
      if (!FLAGS_i.compare("Image")) {
        cv::waitKey(0);
      }
    }
    // enable = false;
    // input_spin.join();
    slog::info << "Execution successful" << slog::endl;
    return 0;
  } catch (const std::exception& error) {
    slog::err << error.what() << slog::endl;
    return 1;
  } catch (...) {
    slog::err << "Unknown/internal exception happened." << slog::endl;
    return 1;
  }
}
