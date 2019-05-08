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
 * @brief a header file with declaration of PipelineFilters class
 * @file pipeline_filters.cpp
 */

#include <vino_param_lib/param_manager.hpp>
#include "dynamic_vino_lib/pipeline_filters.hpp"


PipelineFilters::PipelineFilters() {}

PipelineFilters::PipelineFilters(
  const Params::ParamManager::PipelineRawData & params)
{
  add(params.filters);
}

void PipelineFilters::add(
  const std::vector<Params::ParamManager::FilterRawData> & filters)
{
  pipeline_filters_ = filters;
}

std::string PipelineFilters::findFilterConditions(
  const std::string & input, const std::string & output)
{
  for (auto filter : pipeline_filters_) {
    if (!input.compare(filter.input) && !output.compare(filter.output)) {
      return filter.filter_conditions;
    }
  }
  return "";
}
