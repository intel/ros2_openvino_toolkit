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
#include "openvino_wrapper_lib/engines/engine_manager.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/models/base_model.hpp"
#include "openvino_wrapper_lib/utils/version_info.hpp"
#include <openvino_param_lib/param_manager.hpp>
#include <openvino/openvino.hpp>

std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine(const std::string& device,
                                                                      const std::shared_ptr<Models::BaseModel>& model)
{
  return createEngine_V2022(device, model);
}

std::shared_ptr<Engines::Engine>
Engines::EngineManager::createEngine_V2022(const std::string& device, const std::shared_ptr<Models::BaseModel>& model)
{
  ov::Core core;
  const std::string compile_device = model->prepareForDevice(device);
  ov::CompiledModel executable_network = core.compile_model(model->getModel(), compile_device);
  ov::InferRequest infer_request = executable_network.create_infer_request();

  auto engine = std::make_shared<Engines::Engine>(infer_request, executable_network, device);
  slog::info << "Engine created: requested device=" << device << ", executing on=" << engine->getDeviceName()
             << slog::endl;
  return engine;
}
