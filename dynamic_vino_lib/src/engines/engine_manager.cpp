// Copyright (c) 2018-2019 Intel Corporation
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
#include "dynamic_vino_lib/engines/engine_manager.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/factory.hpp"
#include "dynamic_vino_lib/models/base_model.hpp"

std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine(
  const std::string & device, const std::shared_ptr<Models::BaseModel> & model)
{
#if(defined(USE_IE_CORE))
  return createEngine_V2019R2_plus(device, model);
#else
  return createEngine_beforeV2019R2(device, model);
#endif
}

std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine_beforeV2019R2(
  const std::string & device, const std::shared_ptr<Models::BaseModel> & model)
{
  if(plugins_for_devices_.find(device) == plugins_for_devices_.end()) {
      std::string empty_l, empty_c;
      plugins_for_devices_[device] =
        *Factory::makePluginByName(device, empty_l, empty_c, false);
      slog::info << "Created plugin for " << device << slog::endl;
  }
  auto executeable_network = 
  plugins_for_devices_[device].LoadNetwork(model->getNetReader()->getNetwork(), {});
  auto request = executeable_network.CreateInferRequestPtr();

  return std::make_shared<Engines::Engine>(request);
}

#if(defined(USE_IE_CORE))
std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine_V2019R2_plus(
  const std::string & device, const std::shared_ptr<Models::BaseModel> & model)
{
  InferenceEngine::Core core;
  auto executable_network = core.LoadNetwork(model->getNetReader()->getNetwork(), device);
  auto request = executable_network.CreateInferRequestPtr();

  return std::make_shared<Engines::Engine>(request);
}
#endif
