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
 * @brief a header file with definition of Engine class
 * @file engine.cpp
 */
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/slog.hpp"

#if(defined(USE_OLD_E_PLUGIN_API))
Engines::Engine::Engine(
  InferenceEngine::InferencePlugin plg,
  const Models::BaseModel::Ptr base_model)
{
  request_ = (plg.LoadNetwork(base_model->getModel()->getNetwork(), {})).CreateInferRequestPtr();
}
#endif

Engines::Engine::Engine(
  ov::InferRequest & request)
{
  request_ = request;
}
