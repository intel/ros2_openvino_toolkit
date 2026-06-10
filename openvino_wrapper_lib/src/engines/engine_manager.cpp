// Copyright (c) 2018-2026 Intel Corporation
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
#include <openvino/runtime/intel_gpu/ocl/va.hpp>
#include <openvino/core/preprocess/pre_post_process.hpp>
#if (defined(USE_OLD_E_PLUGIN_API))
#include <extension/ext_list.hpp>
#endif

std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine(const std::string& device,
                                                                      const std::shared_ptr<Models::BaseModel>& model)
{
#if (defined(USE_OLD_E_PLUGIN_API))
  return createEngine_beforeV2019R2(device, model);
#else
  return createEngine_V2022(device, model);
#endif
}

std::shared_ptr<Engines::Engine>
Engines::EngineManager::createEngine_V2022(const std::string& device, const std::shared_ptr<Models::BaseModel>& model)
{
  ov::Core core;
  ov::CompiledModel executable_network = core.compile_model(model->getModel(), device);
  ov::InferRequest infer_request = executable_network.create_infer_request();

  return std::make_shared<Engines::Engine>(infer_request);
}

std::shared_ptr<Engines::Engine>
Engines::EngineManager::createVaEngine(const std::string& device,
                                       const std::string& model_path,
                                       int net_h, int net_w,
                                       VADisplay va_dpy)
{
  if (net_h <= 0 || net_w <= 0) {
    throw std::runtime_error(
      "createVaEngine: model input dimensions are zero — ensure the inference "
      "is fully initialized before calling createVaEngine");
  }

  // Use a dedicated ov::Core for the VA engine — separate from any CPU/normal
  // GPU engines so contexts don't conflict.
  ov::Core va_core;
  ov::intel_gpu::ocl::VAContext va_ctx(va_core, va_dpy);

  // For NV12_TWO_PLANES PPP the model's *network* input shape stays as the
  // original NCHW [1,3,H,W] exported by ONNX — PPP injects the NV12 tensors
  // before it.  Do NOT reshape to NHWC [1,H,W,3] here.
  // (The dynamic-shape model is already [1,3,-1,-1]; fix spatial dims only.)
  auto raw_model = va_core.read_model(model_path);
  raw_model->reshape({{ raw_model->input().get_any_name(),
                        ov::PartialShape{1, 3, net_h, net_w} }});

  ov::preprocess::PrePostProcessor ppp(raw_model);

  // Two-plane NV12 VA surface — VEBOX/SFC pre-scales to net_h×net_w via
  // InferRequirements so no resize is needed (OV GPU plugin doesn't support
  // resize() for GPU_SURFACE tensors).
  ppp.input()
    .tensor()
    .set_element_type(ov::element::u8)
    .set_color_format(ov::preprocess::ColorFormat::NV12_TWO_PLANES, { "y", "uv" })
    .set_memory_type(ov::intel_gpu::memory_type::surface);

  ppp.input().preprocess()
    .convert_color(ov::preprocess::ColorFormat::BGR);

  ppp.input().model().set_layout("NCHW");

  ppp.output().tensor().set_element_type(ov::element::f32);

  auto built_model = ppp.build();

  // Compile against the VAContext so OV uses the VA display's OpenCL queue.
  ov::CompiledModel compiled = va_core.compile_model(built_model, va_ctx);
  slog::info << "[EngineManager] VA surface engine compiled for " << device
             << " — input " << net_w << "×" << net_h << " NV12 GPU_SURFACE" << slog::endl;

  // Store va_core inside Engine so the GPU plugin and VA context stay alive
  // for the lifetime of inference.  Destroying va_core would tear down the
  // GPU plugin context and cause SIGSEGV inside infer() after ~200+ frames.
  return std::make_shared<Engines::Engine>(std::move(va_core), std::move(compiled));
}

#if (defined(USE_OLD_E_PLUGIN_API))
std::shared_ptr<Engines::Engine> Engines::EngineManager::createEngine_beforeV2019R2(
    const std::string& device, const std::shared_ptr<Models::BaseModel>& model)
{
  if (plugins_for_devices_.find(device) == plugins_for_devices_.end()) {
    auto pcommon = Params::ParamManager::getInstance().getCommon();
    plugins_for_devices_[device] = *makePluginByName(device, pcommon.custom_cpu_library, pcommon.custom_cldnn_library,
                                                     pcommon.enable_performance_count);
    slog::info << "Created plugin for " << device << slog::endl;
  }

  auto executeable_network = plugins_for_devices_[device].LoadNetwork(model->getModel()->getNetwork(), {});
  auto request = executeable_network.CreateInferRequestPtr();

  return std::make_shared<Engines::Engine>(request);
}

std::unique_ptr<InferenceEngine::InferencePlugin>
Engines::EngineManager::makePluginByName(const std::string& device_name, const std::string& custom_cpu_library_message,
                                         const std::string& custom_cldnn_message, bool performance_message)
{
  slog::info << "Creating plugin for " << device_name << slog::endl;

  InferenceEngine::InferencePlugin plugin =
      InferenceEngine::PluginDispatcher({ "../../../lib/intel64", "" }).getPluginByDevice(device_name);

  /** Printing plugin version **/
  printPluginVersion(plugin, std::cout);

  /** Load extensions for the CPU plugin **/
  if ((device_name.find("CPU") != std::string::npos)) {
    plugin.AddExtension(std::make_shared<InferenceEngine::Extensions::Cpu::CpuExtensions>());
    if (!custom_cpu_library_message.empty()) {
      slog::info << "custom cpu library is not empty, tyring to use this extension:" << custom_cpu_library_message
                 << slog::endl;
      // CPU(MKLDNN) extensions are loaded as a shared library and passed as a
      // pointer to base
      // extension
      auto extension_ptr = InferenceEngine::make_so_pointer<InferenceEngine::IExtension>(custom_cpu_library_message);
      plugin.AddExtension(extension_ptr);
    }
  } else if (!custom_cldnn_message.empty()) {
    slog::info << "custom cldnn library is not empty, tyring to use this extension:" << custom_cldnn_message
               << slog::endl;
    // Load Extensions for other plugins not CPU
    plugin.SetConfig({ { InferenceEngine::PluginConfigParams::KEY_CONFIG_FILE, custom_cldnn_message } });
  }
  if (performance_message) {
    plugin.SetConfig(
        { { InferenceEngine::PluginConfigParams::KEY_PERF_COUNT, InferenceEngine::PluginConfigParams::YES } });
  }

  return std::make_unique<InferenceEngine::InferencePlugin>(InferenceEngine::InferenceEnginePluginPtr(plugin));
}
#endif
