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
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.hpp
 */
#ifndef DYNAMIC_VINO_LIB__PIPELINE_MANAGER_HPP_
#define DYNAMIC_VINO_LIB__PIPELINE_MANAGER_HPP_

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline.hpp"

/**
 * @class PipelineManager
 * @brief This class manages the lifecycles of pipelines.
 */
class PipelineManager {
 public:
  /**
  * @brief Get the singleton instance of PipelineManager class.
  * The instance will be created when first call.
  * @return The reference of PipelineManager instance.
  */
  static PipelineManager& getInstance() {
    static PipelineManager manager_;
    return manager_;
  };

  std::shared_ptr<Pipeline> createPipeline(
      const Params::ParamManager::PipelineParams& params);
  void removePipeline(const std::string& name);
  PipelineManager& updatePipeline(
      const std::string& name,
      const Params::ParamManager::PipelineParams& params);

  void runAll();
  void stopAll();
  void joinAll();

  enum PipelineState {
    PipelineState_ThreadNotCreated,
    PipelineState_ThreadStopped,
    PipelineState_ThreadRunning,
    PipelineState_Error
  };
  struct PipelineData {
    Params::ParamManager::PipelineParams params;
    std::shared_ptr<Pipeline> pipeline;
    std::vector<std::shared_ptr<rclcpp::Node>> spin_nodes;
    std::shared_ptr<std::thread> thread;
    PipelineState state;
  };

 private:
  PipelineManager(){};
  PipelineManager(PipelineManager const&);
  void operator=(PipelineManager const&);
  void threadPipeline(const char* name);
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
  parseInputDevice(const Params::ParamManager::PipelineParams& params);
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> parseOutput(
      const Params::ParamManager::PipelineParams& params);
  std::map<std::string, std::shared_ptr<dynamic_vino_lib::BaseInference>>
  parseInference(const Params::ParamManager::PipelineParams& params);
  std::shared_ptr<dynamic_vino_lib::BaseInference> createFaceDetection(
      const Params::ParamManager::InferenceParams& infer);
  std::shared_ptr<dynamic_vino_lib::BaseInference> createAgeGenderRecognition(
      const Params::ParamManager::InferenceParams& infer);
  std::shared_ptr<dynamic_vino_lib::BaseInference> createEmotionRecognition(
      const Params::ParamManager::InferenceParams& infer);
  std::shared_ptr<dynamic_vino_lib::BaseInference> createHeadPoseEstimation(
      const Params::ParamManager::InferenceParams& infer);
  std::shared_ptr<dynamic_vino_lib::BaseInference> createObjectDetection(
      const Params::ParamManager::InferenceParams& infer);
  std::shared_ptr<dynamic_vino_lib::BaseInference> createObjectSegmentation(
      const Params::ParamManager::InferenceParams& infer);

  std::map<std::string, PipelineData> pipelines_;
  std::map<std::string, InferenceEngine::InferencePlugin> plugins_for_devices_;
};

#endif  // DYNAMIC_VINO_LIB__PIPELINE_MANAGER_HPP_