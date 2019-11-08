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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <ament_index_cpp/get_resource.hpp>
#include <vino_param_lib/param_manager.hpp>

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

#include "dynamic_vino_lib/pipeline.hpp"
#include "dynamic_vino_lib/pipeline_manager.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "extension/ext_list.hpp"
//#include "gflags/gflags.h"
#include "inference_engine.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

std::string getConfigPath(std::string config_file)
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "dynamic_vino_sample", content, &prefix_path);
  return prefix_path + "/share/dynamic_vino_sample/param/testParam/param/" + config_file;
}

TEST(UnitTestCheckPipeline, testCreatePipeline)
{
  std::vector<std::string> config_files = {"image_object_service_test.yaml", "pipeline_face_reid_video.yaml",
	                                   "pipeline_image_test.yaml", "pipeline_reidentification_test.yaml",
					   "pipeline_vehicle_detection_test.yaml", "image_people_service_test.yaml",
					   "pipeline_face_test.yaml", "pipeline_segmentation_test.yaml",
                                           "pipeline_object_yolo_test.yaml"};
  for (unsigned int i = 0; i < config_files.size(); i++)
  {
    std::string config_file = getConfigPath(config_files[i]);
    EXPECT_TRUE(std::ifstream(config_file).is_open());
    ASSERT_NO_THROW({
      Params::ParamManager::getInstance().parse(config_file);
      auto pipelines = Params::ParamManager::getInstance().getPipelines();
      EXPECT_GT(pipelines.size(), 0);

      for (auto & p : pipelines) {
        PipelineManager::getInstance().createPipeline(p);
      }
    });
  }
}

TEST(UnitTestCheckPipeline, testPipelineIncorrectConfig)
{
  std::string config_file = getConfigPath("pipeline_ianormal.yaml");
  EXPECT_TRUE(std::ifstream(config_file).is_open());
  try{
    Params::ParamManager::getInstance().parse(config_file);
    auto pipelines = Params::ParamManager::getInstance().getPipelines();
    EXPECT_GT(pipelines.size(), 0);

    for (auto & p : pipelines) {
      PipelineManager::getInstance().createPipeline(p);
    }
  }
  catch (...) {
    SUCCEED();
  }
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
