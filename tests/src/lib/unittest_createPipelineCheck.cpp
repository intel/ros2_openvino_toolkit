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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <openvino_param_lib/param_manager.hpp>

#include <unistd.h>
#include <cstdio>
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

#include "openvino_wrapper_lib/pipeline.hpp"
#include "openvino_wrapper_lib/pipeline_manager.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// Returns the path of a copy of the installed test YAML whose path placeholders
// (<OPENVINO_NODE_SHARE>, <OPENVINO_DATA>, <OPENVINO_TEST_DATA>) are resolved by
// the same helper the launch files use. Returns the unresolved file on failure.
std::string getConfigPath(std::string config_file)
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "openvino_test", content, &prefix_path);
  std::string original = prefix_path + "/share/openvino_test/param/" + config_file;

  const std::string launch_dir = ament_index_cpp::get_package_share_directory("openvino_node") + "/launch";
  const std::string cmd = "python3 -c \"import sys; sys.path.insert(0, '" + launch_dir +
                          "'); import launch_helpers; print(launch_helpers.resolve_yaml_paths('" + config_file +
                          "', 'openvino_test'))\" 2>/dev/null";
  FILE* pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    return original;
  }
  std::string resolved;
  char line[4096];
  while (fgets(line, sizeof(line), pipe) != nullptr) {
    resolved = line;
  }
  pclose(pipe);
  while (!resolved.empty() && (resolved.back() == '\n' || resolved.back() == '\r')) {
    resolved.pop_back();
  }
  return resolved.empty() ? original : resolved;
}

TEST(UnitTestCheckPipeline, testCreatePipeline)
{
  auto node = rclcpp::Node::make_shared("pipeline_test");
  std::vector<std::string> config_files = { "image_object_service_test.yaml",
                                            "pipeline_face_reid_video.yaml",
                                            "pipeline_image_test.yaml",
                                            "pipeline_reidentification_test.yaml",
                                            "pipeline_vehicle_detection_test.yaml",
                                            "image_people_service_test.yaml",
                                            "pipeline_segmentation_test.yaml",
                                            "pipeline_face_test.yaml" };
  for (unsigned int i = 0; i < config_files.size(); i++) {
    std::string config_file = getConfigPath(config_files[i]);
    EXPECT_TRUE(std::ifstream(config_file).is_open());
    ASSERT_NO_THROW({
      Params::ParamManager::getInstance().parse(config_file);
      auto pipelines = Params::ParamManager::getInstance().getPipelines();
      EXPECT_GT(pipelines.size(), 0);

      for (auto& p : pipelines) {
        PipelineManager::getInstance().createPipeline(p, node);
      }
    });
  }
}

TEST(UnitTestCheckPipeline, testCreatePipelineRealsense)
{
  auto node = rclcpp::Node::make_shared("pipeline_realsense_test");
  std::string config_files = "pipeline_object_yolo_test.yaml";
  std::string config_file = getConfigPath(config_files);
  EXPECT_TRUE(std::ifstream(config_file).is_open());

  // YOLOv5 has no pre-converted IR download; skip when the model was not converted locally.
  std::ifstream yaml(config_file);
  std::string line;
  while (std::getline(yaml, line)) {
    const auto pos = line.find("model:");
    if (pos != std::string::npos && line.find("model_type:") == std::string::npos) {
      std::string model = line.substr(pos + 6);
      model.erase(0, model.find_first_not_of(" \t"));
      if (!std::ifstream(model).good()) {
        GTEST_SKIP() << "Model not found, convert it first: " << model;
      }
    }
  }
  ASSERT_NO_THROW({
    Params::ParamManager::getInstance().parse(config_file);
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    EXPECT_GT(pipelines.size(), 0);

    for (auto& p : pipelines) {
      PipelineManager::getInstance().createPipeline(p, node);
    }
  });
}

TEST(UnitTestCheckPipeline, testPipelineIncorrectConfig)
{
  auto node = rclcpp::Node::make_shared("pipeline_anormal_test");
  std::string config_file = getConfigPath("pipeline_anormal.yaml");
  EXPECT_TRUE(std::ifstream(config_file).is_open());
  try {
    Params::ParamManager::getInstance().parse(config_file);
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    EXPECT_GT(pipelines.size(), 0);

    for (auto& p : pipelines) {
      PipelineManager::getInstance().createPipeline(p, node);
    }
  } catch (...) {
    SUCCEED();
  }
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
