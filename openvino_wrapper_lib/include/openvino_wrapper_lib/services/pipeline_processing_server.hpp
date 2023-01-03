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
#ifndef OPENVINO_WRAPPER_LIB__SERVICES__PIPELINE_PROCESSING_SERVER_HPP_
#define OPENVINO_WRAPPER_LIB__SERVICES__PIPELINE_PROCESSING_SERVER_HPP_

#include <openvino_msgs/msg/connection.hpp>
#include <openvino_msgs/msg/pipeline_request.hpp>
#include <openvino_msgs/msg/pipeline.hpp>
#include <openvino_msgs/srv/pipeline_srv.hpp>
#include <openvino_wrapper_lib/pipeline_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <string>
#include <map>

namespace vino_service
{
template<typename T>
class PipelineProcessingServer : public rclcpp::Node
{
public:
  explicit PipelineProcessingServer(
    const std::string & service_name);

private:
  void initPipelineService();

  void cbService(
    const std::shared_ptr<typename T::Request> request,
    std::shared_ptr<typename T::Response> response);

  void setResponse(
    std::shared_ptr<typename T::Response> response);

  void setPipelineByRequest(std::string pipeline_name, PipelineManager::PipelineState state);

  std::shared_ptr<rclcpp::Service<T>> service_;
  std::map<std::string, PipelineManager::PipelineData> * pipelines_;
  std::string service_name_;
};
}  // namespace vino_service
#endif  // OPENVINO_WRAPPER_LIB__SERVICES__PIPELINE_PROCESSING_SERVER_HPP_
