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
#if(defined(USE_OLD_E_PLUGIN_API))
#include <extension/ext_list.hpp>
#endif

std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine(
  const std::string & device, const std::shared_ptr<Models::BaseModel> & model)
{
#if(defined(USE_OLD_E_PLUGIN_API))
  return createEngine_beforeV2019R2(device, model);
#else
  return createEngine_V2022(device, model);
#endif
}

std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine_V2022(
  const std::string & device, const std::shared_ptr<Models::BaseModel> & model)
{
  ov::Core core;
  ov::CompiledModel executable_network = core.compile_model(model->getModel(), device);
  ov::InferRequest infer_request = executable_network.create_infer_request();

  return std::make_shared<Engines::Engine>(infer_request);
}

#if(defined(USE_OLD_E_PLUGIN_API))
std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine_beforeV2019R2(
  const std::string & device, const std::shared_ptr<Models::BaseModel> & model)
{
  if(plugins_for_devices_.find(device) == plugins_for_devices_.end()) {
      auto pcommon = Params::ParamManager::getInstance().getCommon();
      plugins_for_devices_[device] = *makePluginByName(device, pcommon.custom_cpu_library,
        pcommon.custom_cldnn_library, pcommon.enable_performance_count);
      slog::info << "Created plugin for " << device << slog::endl;
  }

  auto executeable_network = 
  plugins_for_devices_[device].LoadNetwork(model->getModel()->getNetwork(), {});
  auto request = executeable_network.CreateInferRequestPtr();

  return std::make_shared<Engines::Engine>(request);
}

std::unique_ptr<InferenceEngine::InferencePlugin>
Engines::EngineManager::makePluginByName(
  const std::string & device_name, const std::string & custom_cpu_library_message,
  const std::string & custom_cldnn_message, bool performance_message)
{
  slog::info << "Creating plugin for " << device_name << slog::endl;

  InferenceEngine::InferencePlugin plugin =
    InferenceEngine::PluginDispatcher({"../../../lib/intel64", ""})
    .getPluginByDevice(device_name);

  /** Printing plugin version **/
  printPluginVersion(plugin, std::cout);

  /** Load extensions for the CPU plugin **/
  if ((device_name.find("CPU") != std::string::npos)) {
    plugin.AddExtension(std::make_shared<InferenceEngine::Extensions::Cpu::CpuExtensions>());
    if (!custom_cpu_library_message.empty()) {
      slog::info << "custom cpu library is not empty, tyring to use this extension:"
        << custom_cpu_library_message << slog::endl;
      // CPU(MKLDNN) extensions are loaded as a shared library and passed as a
      // pointer to base
      // extension
      auto extension_ptr =
        InferenceEngine::make_so_pointer<InferenceEngine::IExtension>(custom_cpu_library_message);
      plugin.AddExtension(extension_ptr);
    }
  } else if (!custom_cldnn_message.empty()) {
    slog::info << "custom cldnn library is not empty, tyring to use this extension:"
        << custom_cldnn_message << slog::endl;
    // Load Extensions for other plugins not CPU
    plugin.SetConfig(
      {{InferenceEngine::PluginConfigParams::KEY_CONFIG_FILE, custom_cldnn_message}});
  }
  if (performance_message) {
    plugin.SetConfig({{InferenceEngine::PluginConfigParams::KEY_PERF_COUNT,
        InferenceEngine::PluginConfigParams::YES}});
  }

  return std::make_unique<InferenceEngine::InferencePlugin>(
    InferenceEngine::InferenceEnginePluginPtr(plugin));
}
#endif
