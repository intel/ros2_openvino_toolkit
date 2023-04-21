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
 * @brief a header file with declaration of Pipeline class
 * @file pipeline.cpp
 */

#include <openvino_param_lib/param_manager.hpp>
#include <memory>
#include <string>
#include <utility>
#include "openvino_wrapper_lib/pipeline_params.hpp"

PipelineParams::PipelineParams(const std::string & name)
{
  params_.name = name;
}

PipelineParams::PipelineParams(const Params::ParamManager::PipelineRawData & params)
{
  params_ = params;
}

PipelineParams & PipelineParams::operator=(const Params::ParamManager::PipelineRawData & params)
{
  params_.name = params.name;
  params_.infers = params.infers;
  params_.inputs = params.inputs;
  params_.outputs = params.outputs;
  params_.connects = params.connects;

  return *this;
}

Params::ParamManager::PipelineRawData PipelineParams::getPipeline(const std::string & name)
{
  return Params::ParamManager::getInstance().getPipeline(name);
}

void PipelineParams::update()
{
  if (!params_.name.empty()) {
    params_ = getPipeline(params_.name);
  }
}

void PipelineParams::update(const Params::ParamManager::PipelineRawData & params)
{
  params_ = params;
}

bool PipelineParams::isOutputTo(std::string & output)
{
  if (std::find(params_.outputs.begin(), params_.outputs.end(), output) != params_.outputs.end()) {
    return true;
  }
  return false;
}

bool PipelineParams::isGetFps()
{
  /**< Only "Image" input can't computing FPS >**/
  if (params_.inputs.size() == 0) {
    return false;
  }
  return std::find(params_.inputs.begin(), params_.inputs.end(), kInputType_Image) ==
         params_.inputs.end();
}

std::string PipelineParams::findFilterConditions(
  const std::string & input, const std::string & output)
{
  for (auto filter : params_.filters) {
    if (!input.compare(filter.input) && !output.compare(filter.output)) {
      return filter.filter_conditions;
    }
  }
  return "";
}
