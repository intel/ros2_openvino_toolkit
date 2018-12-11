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
    showUsage();
    return false;
  }
  slog::info << "Parsing input parameters" << slog::endl;
  if (FLAGS_i.empty()) {
    throw std::logic_error("Parameter -i is not set");
  }
  if (!FLAGS_i.compare("Video") && FLAGS_i_path.empty()) {
    throw std::logic_error("Parameter -i_path is not set");
  }
  if (!FLAGS_i.compare("Image") && FLAGS_i_path.empty()) {
    throw std::logic_error("Parameter -i_path is not set");
  }
  if (FLAGS_m.empty()) {
    throw std::logic_error("Parameter -m is not set");
  }
  if (FLAGS_n_ag < 1) {
    throw std::logic_error("Parameter -n_ag cannot be 0");
  }
  if (FLAGS_n_hp < 1) {
    throw std::logic_error("Parameter -n_hp cannot be 0");
  }
  return true;
}

void thread_op(std::shared_ptr<rclcpp::Node> node, bool& enable) {
  slog::info << "In Input Spin Thread (enable: " << enable << ")" << slog::endl;
  while (enable) {
    // slog::info << ".";
    rclcpp::spin_some(node);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // std::shared_ptr<rclcpp::Node> node =
  // rclcpp::Node::make_shared("dynamic_vino_sample");

  try {
    std::cout << "InferenceEngine: " << GetInferenceEngineVersion()
              << std::endl;

    // ------------------------------ Parsing and validation of input args
    // -------------------------
    if (!parseAndCheckCommandLine(argc, argv)) {
      return 0;
    }

    // --------------------------- 1. Load Plugin for inference engine
    // -----------------------------
    std::map<std::string, InferencePlugin> plugins_for_devices;
    std::vector<std::pair<std::string, std::string>> cmd_options = {
        {FLAGS_d, FLAGS_m},
        {FLAGS_d_ag, FLAGS_m_ag},
        {FLAGS_d_hp, FLAGS_m_hp},
        {FLAGS_d_em, FLAGS_m_em}};
    slog::info << "device_FACE:" << FLAGS_d << slog::endl;
    slog::info << "model_FACE:" << FLAGS_m << slog::endl;
    slog::info << "device_AG:" << FLAGS_d_ag << slog::endl;
    slog::info << "model_AG:" << FLAGS_m_ag << slog::endl;
    slog::info << "model_HeadPose:" << FLAGS_m_hp << slog::endl;
    slog::info << "device_HeadPose:" << FLAGS_d_hp << slog::endl;

    for (auto&& option : cmd_options) {
      auto device_name = option.first;
      auto network_name = option.second;
      if (device_name.empty() || network_name.empty()) {
        continue;
      }
      if (plugins_for_devices.find(device_name) != plugins_for_devices.end()) {
        continue;
      }
      plugins_for_devices[device_name] =
          *Factory::makePluginByName(device_name, FLAGS_l, FLAGS_c, FLAGS_pc);
    }

    // --------------------------- 2. Generate Input Device and Output
    // Device-----------------------
    slog::info << "Reading input" << slog::endl;
    auto input_ptr = Factory::makeInputDeviceByName(FLAGS_i, FLAGS_i_path);
    // auto input_ptr = std::make_shared<Input::RealSenseCameraTopic>();

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
    auto face_detection_model =
        std::make_shared<Models::FaceDetectionModel>(FLAGS_m, 1, 1, 1);
    face_detection_model->modelInit();
    auto face_detection_engine = std::make_shared<Engines::Engine>(
        plugins_for_devices[FLAGS_d], face_detection_model);
    auto face_inference_ptr =
        std::make_shared<dynamic_vino_lib::FaceDetection>(FLAGS_t);
    face_inference_ptr->loadNetwork(face_detection_model);
    face_inference_ptr->loadEngine(face_detection_engine);

    // generate emotions detection inference
    auto emotions_detection_model =
        std::make_shared<Models::EmotionDetectionModel>(FLAGS_m_em, 1, 1, 16);
    emotions_detection_model->modelInit();
    auto emotions_detection_engine = std::make_shared<Engines::Engine>(
        plugins_for_devices[FLAGS_d_em], emotions_detection_model);
    auto emotions_inference_ptr =
        std::make_shared<dynamic_vino_lib::EmotionsDetection>();
    emotions_inference_ptr->loadNetwork(emotions_detection_model);
    emotions_inference_ptr->loadEngine(emotions_detection_engine);

    // generate age gender detection inference
    auto agegender_detection_model =
        std::make_shared<Models::AgeGenderDetectionModel>(FLAGS_m_ag, 1, 2, 16);
    agegender_detection_model->modelInit();
    auto agegender_detection_engine = std::make_shared<Engines::Engine>(
        plugins_for_devices[FLAGS_d_ag], agegender_detection_model);
    auto agegender_inference_ptr =
        std::make_shared<dynamic_vino_lib::AgeGenderDetection>();
    agegender_inference_ptr->loadNetwork(agegender_detection_model);
    agegender_inference_ptr->loadEngine(agegender_detection_engine);

    // generate head pose estimation inference

    auto headpose_detection_network =
        std::make_shared<Models::HeadPoseDetectionModel>(FLAGS_m_hp, 1, 3, 16);
    headpose_detection_network->modelInit();
    auto headpose_detection_engine = std::make_shared<Engines::Engine>(
        plugins_for_devices[FLAGS_d_hp], headpose_detection_network);
    auto headpose_inference_ptr =
        std::make_shared<dynamic_vino_lib::HeadPoseDetection>();
    headpose_inference_ptr->loadNetwork(headpose_detection_network);
    headpose_inference_ptr->loadEngine(headpose_detection_engine);

    // --------------------------- 4. Build Pipeline
    // --------------------------------------
    Pipeline pipe;
    // pipe.add("video_input", std::move(input_ptr));
    pipe.add("video_input", input_ptr);
    pipe.add("video_input", "face_detection", face_inference_ptr);
    pipe.add("face_detection", "emotions_detection", emotions_inference_ptr);
    pipe.add("face_detection", "age_gender_detection", agegender_inference_ptr);
    pipe.add("face_detection", "headpose_detection", headpose_inference_ptr);
    pipe.add("emotions_detection", "video_output", output_ptr);
    pipe.add("age_gender_detection", "video_output", output_ptr);
    pipe.add("headpose_detection", "video_output", output_ptr);
    pipe.add("face_detection", "video_output", output_ptr);
    auto ros_topic_output_ptr = std::make_shared<Outputs::RosTopicOutput>();
    pipe.add("face_detection", "ros_output", ros_topic_output_ptr);
    pipe.add("emotions_detection", "ros_output", ros_topic_output_ptr);
    pipe.add("age_gender_detection", "ros_output", ros_topic_output_ptr);
    pipe.add("headpose_detection", "ros_output", ros_topic_output_ptr);
    pipe.setCallback();
    pipe.printPipeline();

    // auto node =  rclcpp::Node::make_shared("sample");
    // bool enable = true;
    // std::thread input_spin(thread_op, input_ptr->getHandler(),
    // std::ref(enable));

    slog::info << "Wait 3s before entering data process..." << slog::endl;
    sleep(3);
    slog::info << "DONE!" << slog::endl;

    // --------------------------- 5. Run Pipeline
    // -----------------------------------------
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
