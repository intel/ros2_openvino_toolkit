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
 * @brief a header file with declaration of Inference Manager class
 * @file inference_manager.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__INFERENCE_MANAGER_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__INFERENCE_MANAGER_HPP_

#include <openvino_param_lib/param_manager.hpp>
#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/pipeline.hpp"

/**
 * @class InferenceManager
 * @brief This class manages inference resources.
 */
class InferenceManager
{
public:
  /**
  * @brief Get the singleton instance of InferenceManager class.
  * The instance will be created when first call.
  * @return The reference of InferenceManager instance.
  */
  static InferenceManager & getInstance()
  {
    static InferenceManager manager_;
    return manager_;
  }

  std::shared_ptr<Pipeline> createPipeline(
    const Params::ParamManager::PipelineRawData & params);
  void removePipeline(const std::string & name);
  InferenceManager & updatePipeline(
    const std::string & name,
    const Params::ParamManager::PipelineRawData & params);

  void runAll();
  void stopAll();
  void joinAll();

  enum PipelineState
  {
    PipelineState_ThreadNotCreated,
    PipelineState_ThreadStopped,
    PipelineState_ThreadRunning,
    PipelineState_Error
  };
  struct PipelineData
  {
    Params::ParamManager::PipelineRawData params;
    std::shared_ptr<Pipeline> pipeline;
    std::vector<std::shared_ptr<rclcpp::Node>> spin_nodes;
    std::shared_ptr<std::thread> thread;
    PipelineState state;
  };

private:
  InferenceManager() {}
  InferenceManager(InferenceManager const &);
  void operator=(InferenceManager const &);
  void threadPipeline(const char * name);
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
  parseInputDevice(const Params::ParamManager::PipelineRawData & params);
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>> parseOutput(
    const Params::ParamManager::PipelineRawData & params);
  std::map<std::string, std::shared_ptr<openvino_wrapper_lib::BaseInference>>
  parseInference(const Params::ParamManager::PipelineRawData & params);
  std::shared_ptr<openvino_wrapper_lib::BaseInference> createFaceDetection(
    const Params::ParamManager::InferenceParams & infer);
  std::shared_ptr<openvino_wrapper_lib::BaseInference> createAgeGenderRecognition(
    const Params::ParamManager::InferenceParams & infer);
  std::shared_ptr<openvino_wrapper_lib::BaseInference> createEmotionRecognition(
    const Params::ParamManager::InferenceParams & infer);
  std::shared_ptr<openvino_wrapper_lib::BaseInference> createHeadPoseEstimation(
    const Params::ParamManager::InferenceParams & infer);
  std::shared_ptr<openvino_wrapper_lib::BaseInference> createObjectDetection(
    const Params::ParamManager::InferenceParams & infer);

  std::map<std::string, PipelineData> pipelines_;
  };

#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__INFERENCE_MANAGER_HPP_
