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

#include "openvino_wrapper_lib/services/pipeline_processing_server.hpp"

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
#include "openvino_wrapper_lib/slog.hpp"

namespace vino_service
{
template<typename T>
PipelineProcessingServer<T>::PipelineProcessingServer(
  const std::string & service_name)
: Node(service_name),
  service_name_(service_name)
{
  pipelines_ = PipelineManager::getInstance().getPipelinesPtr();
  initPipelineService();
}

template<typename T>
void PipelineProcessingServer<T>::initPipelineService()
{
  service_ = create_service<T>("/openvino_toolkit/pipeline_service",
      std::bind(&PipelineProcessingServer::cbService, this,
      std::placeholders::_1, std::placeholders::_2));
}

template<typename T>
void PipelineProcessingServer<T>::setResponse(
  std::shared_ptr<typename T::Response> response)
{
  for (auto it = pipelines_->begin(); it != pipelines_->end(); ++it) {
    openvino_msgs::msg::Pipeline pipeline_msg;
    pipeline_msg.name = it->first;
    pipeline_msg.running_status = std::to_string(it->second.state);

    auto connection_map = it->second.pipeline->getPipelineDetail();
    for (auto & current_pipe : connection_map) {
      openvino_msgs::msg::Connection connection;
      connection.input = current_pipe.first.c_str();
      connection.output = current_pipe.second.c_str();
      pipeline_msg.connections.push_back(connection);
    }
    response->pipelines.push_back(pipeline_msg);
  }
}
template<typename T>
void PipelineProcessingServer<T>::setPipelineByRequest(
  std::string pipeline_name,
  PipelineManager::PipelineState state)
{
  for (auto it = pipelines_->begin(); it != pipelines_->end(); ++it) {
    if (pipeline_name == it->first) {
      std::cout << pipeline_name << "set :" << state << std::endl;
      it->second.state = state;
    }
  }
}

template<typename T>
void PipelineProcessingServer<T>::cbService(
  const std::shared_ptr<typename T::Request> request,
  std::shared_ptr<typename T::Response> response)
{
  std::string req_cmd = request->pipeline_request.cmd;
  std::string req_val = request->pipeline_request.value;
  slog::info << "[PipelineProcessingServer] Pipeline Service get request cmd: " << req_cmd <<
    " val:" << req_val << slog::endl;
  // Todo set initial state by current state
  PipelineManager::PipelineState state = PipelineManager::PipelineState_ThreadRunning;
  if (req_cmd != "GET_PIPELINE") {
    if (req_cmd == "STOP_PIPELINE") {
      state = PipelineManager::PipelineState_ThreadStopped;
    } else if (req_cmd == "RUN_PIPELINE") {
      state = PipelineManager::PipelineState_ThreadRunning;
    } else if (req_cmd == "PAUSE_PIPELINE") {state = PipelineManager::PipelineState_ThreadPasued;}
    setPipelineByRequest(req_val, state);
  }
  setResponse(response);
}
template class PipelineProcessingServer<openvino_msgs::srv::PipelineSrv>;
}  // namespace vino_service
