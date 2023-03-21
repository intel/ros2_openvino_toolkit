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

#include "openvino_wrapper_lib/services/frame_processing_server.hpp"
#include <object_msgs/srv/people.hpp>
#include <object_msgs/srv/detect_object.hpp>

#include <ament_index_cpp/get_resource.hpp>
#include <openvino_param_lib/param_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <map>
#include <chrono>
#include <thread>

#include "openvino_wrapper_lib/pipeline_manager.hpp"
#include "openvino_wrapper_lib/pipeline.hpp"
#include "openvino_wrapper_lib/inputs/base_input.hpp"
#include "openvino_wrapper_lib/inputs/image_input.hpp"
#include "openvino_wrapper_lib/slog.hpp"

namespace vino_service
{
template<typename T>
FrameProcessingServer<T>::FrameProcessingServer(
  const std::string & service_name,
  const std::string & config_path)
: Node("node_with_service"),
  service_name_(service_name),
  config_path_(config_path)
{
  initService(config_path);
}

template<typename T>
void FrameProcessingServer<T>::initService(
  const std::string & config_path)
{
  Params::ParamManager::getInstance().parse(config_path);
  Params::ParamManager::getInstance().print();
  auto pipelines = Params::ParamManager::getInstance().getPipelines();

  if (pipelines.size() != 1) {
    throw std::logic_error("1 and only 1 pipeline can be set to FrameProcessServer!");
  }

  for (auto & p : pipelines) {
    PipelineManager::getInstance().createPipeline(p);
  }

  service_ = create_service<T>("/openvino_toolkit/service",
      std::bind(&FrameProcessingServer::cbService, this,
      std::placeholders::_1, std::placeholders::_2));
}

template<typename T>
void FrameProcessingServer<T>::cbService(
  const std::shared_ptr<typename T::Request> request,
  std::shared_ptr<typename T::Response> response)
{
  std::map<std::string, PipelineManager::PipelineData> pipelines_ =
    PipelineManager::getInstance().getPipelines();
  for (auto it = pipelines_.begin(); it != pipelines_.end(); ++it) {
    PipelineManager::PipelineData & p = pipelines_[it->second.params.name.c_str()];
    auto input = p.pipeline->getInputDevice();
    Input::Config config;
    config.path = request->image_path;
    input->config(config);
    p.pipeline->runOnce();
    auto output_handle = p.pipeline->getOutputHandle();

    for (auto & pair : output_handle) {
      if (!pair.first.compare(kOutputTpye_RosService)) {
        pair.second->setServiceResponse(response);
        pair.second->clearData();
        return;  // TODO(weizhi) , return directly, suppose only 1 pipeline dealing with 1 request.
      }
    }
  }
  slog::info << "[FrameProcessingServer] Callback finished!" << slog::endl;
}

template class FrameProcessingServer<object_msgs::srv::DetectObject>;
template class FrameProcessingServer<object_msgs::srv::People>;
}  // namespace vino_service
