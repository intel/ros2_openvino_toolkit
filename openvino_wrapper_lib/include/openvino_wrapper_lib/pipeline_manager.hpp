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
 * @brief a header file with declaration of Pipeline Manager class
 * @file pipeline_manager.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__PIPELINE_MANAGER_HPP_
#define OPENVINO_WRAPPER_LIB__PIPELINE_MANAGER_HPP_

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
#include "openvino_wrapper_lib/engines/engine_manager.hpp"
#include "openvino_wrapper_lib/vino_factory.hpp"


#define REG_INPUT_FACTORY     VinoFactory<std::string, Input::BaseInputDevice>
#define REG_INPUT(BASE, key, name)  static REG_INPUT_FACTORY::TReg<Input::BASE> gs_input_##name(key)

#define REG_MODEL_FACTORY         VinoFactory<std::string, Models::BaseModel>
#define REG_MODEL(BASE, key, name)     static REG_MODEL_FACTORY::TReg<Models::BASE> gs_model_##name(key)
#define REG_MODEL_type(BASE, key, suffix, name)     static REG_MODEL_FACTORY::TReg<Models::BASE> gs_model_##name(key##suffix)

#define REG_INFERENCE_FACTORY  VinoFactory<std::string, openvino_wrapper_lib::BaseInference>
#define REG_INFERENCE(T, key, name)  static REG_INFERENCE_FACTORY::TReg<openvino_wrapper_lib::T> gs_inference_##name(key)

#define REG_OUTPUT_FACTORY    VinoFactory<std::string, Outputs::BaseOutput>
#define REG_OUTPUT(BASE, key, name) static REG_OUTPUT_FACTORY::TReg<Outputs::BASE> gs_output_##name(key)

// REG_INPUT(class name,         search key(in this case is string type),             expand variable suffix)
// eg: 
// #define REG_INPUT_FACTORY     VinoFactory<std::string, Input::BaseInputDevice>
// #define REG_INPUT(BASE, key, name)  static REG_INPUT_FACTORY::TReg<Input::BASE> gs_input_##name(key)

// REG_INPUT(ImageClass, "JPG", jpg)
// REG_INPUT(ImageClass, "PNG", png)
//         VinoFactory<std::string, Input::BaseInputDevice>::TReg<Input::BASE> gs_input_IMG
//              
//         Viriable type:      VinoFactory<std::string, Input::BaseInputDevice>::TReg<Input::ImageClass>
//         Viriable name:      gs_input_jpg,    gs_input_png
//         search keys:       "JPG",            "PNG"



/**
 * @class PipelineManager
 * @brief This class manages the lifecycles of pipelines.
 */
class PipelineManager
{
public:
  /**
  * @brief Get the singleton instance of PipelineManager class.
  * The instance will be created when first call.
  * @return The reference of PipelineManager instance.
  */
  static PipelineManager & getInstance()
  {
    static PipelineManager manager_;
    return manager_;
  }

  std::shared_ptr<Pipeline> createPipeline(
    const Params::ParamManager::PipelineRawData & params,
    rclcpp::Node::SharedPtr node = nullptr);

  void removePipeline(const std::string & name);
  PipelineManager & updatePipeline(
    const std::string & name,
    const Params::ParamManager::PipelineRawData & params);

  void runAll();
  void stopAll();
  void joinAll();
  void runService();

  enum PipelineState
  {
    PipelineState_ThreadNotCreated = 0,
    PipelineState_ThreadStopped = 1,
    PipelineState_ThreadRunning = 2,
    PipelineState_ThreadPasued = 3,
    PipelineState_Error = 4
  };
  struct PipelineData
  {
    Params::ParamManager::PipelineRawData params;
    std::shared_ptr<Pipeline> pipeline;
    rclcpp::Node::SharedPtr parent_node = nullptr;
    std::vector<std::shared_ptr<rclcpp::Node>> spin_nodes;
    std::shared_ptr<std::thread> thread;
    std::shared_ptr<std::thread> thread_spin_nodes;
    PipelineState state;
  };

  struct ServiceData
  {
    std::shared_ptr<std::thread> thread;
    PipelineState state;
  };

  std::map<std::string, PipelineData> getPipelines()
  {
    return pipelines_;
  }

  std::map<std::string, PipelineData> * getPipelinesPtr()
  {
    return &pipelines_;
  }

private:
  PipelineManager()
  {
  }
  PipelineManager(PipelineManager const &);
  void operator=(PipelineManager const &);
  void threadPipeline(const char * name);
  void threadSpinNodes(const char * name);
  std::map<std::string, std::shared_ptr<Input::BaseInputDevice>>
  parseInputDevice(const PipelineData & params);
  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>>
  parseOutput(const PipelineData & pdata);
  std::map<std::string, std::shared_ptr<openvino_wrapper_lib::BaseInference>>
  parseInference(const Params::ParamManager::PipelineRawData & params);
  std::map<std::string, PipelineData> pipelines_;
  ServiceData service_;
  Engines::EngineManager engine_manager_;
};

#endif  // OPENVINO_WRAPPER_LIB__PIPELINE_MANAGER_HPP_
