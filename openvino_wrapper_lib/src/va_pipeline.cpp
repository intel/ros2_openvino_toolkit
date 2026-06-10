// Copyright (c) 2026 Intel Corporation
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

#include "openvino_wrapper_lib/va_pipeline.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include <opencv2/opencv.hpp>

VaPipeline::VaPipeline(const std::string& name)
  : Pipeline(name)
{
}

void VaPipeline::addVaInference(
    const std::string& name,
    std::shared_ptr<openvino_wrapper_lib::VaSurfaceInference> inf)
{
  va_inferences_[name] = std::move(inf);
}

Input::VaSurfaceTopic* VaPipeline::vaInput()
{
  auto* raw = dynamic_cast<Input::VaSurfaceTopic*>(getInputDevice().get());
  return raw;
}

void VaPipeline::compileVaEngines(const std::string& device, VADisplay va_dpy)
{
  Engines::EngineManager mgr;
  (void)device; (void)va_dpy;  // Callers compile engines manually via createVaEngine().

  // Publish InferRequirements so the camera VEBOX/SFC reconfigures its output.
  auto* va_in = vaInput();
  if (!va_in) {
    slog::warn << "VaPipeline::compileVaEngines: input is not VaSurfaceTopic"
               << slog::endl;
    return;
  }
  uint32_t req_w = static_cast<uint32_t>(va_in->getWidth());
  uint32_t req_h = static_cast<uint32_t>(va_in->getHeight());
  if (req_w > 0 && req_h > 0) {
    va_in->publishRequirements(req_w, req_h, "NV12");
  }
}

void VaPipeline::runOnce()
{
  auto* va_in = vaInput();
  if (!va_in) {
    Pipeline::runOnce();
    return;
  }

  openvino_wrapper_lib::VaSurfaceHolder holder;
  if (!va_in->readVaSurface(&holder)) return;

  // Enqueue + submit for every registered VA inference.
  for (auto& [name, inf_ptr] : va_inferences_) {
    if (holder.hasVaSurface()) {
      fprintf(stderr, "[VaPipeline] enqueueVaSurface surf_id=%u %ux%u\n",
              holder.va_surface_id, holder.width, holder.height);
      if (!inf_ptr->enqueueVaSurface(holder)) {
        slog::warn << "VaPipeline: enqueueVaSurface failed for " << name << slog::endl;
        continue;
      }
      fprintf(stderr, "[VaPipeline] enqueueVaSurface OK — calling submitRequest\n");
    } else if (!holder.cpu_mat.empty()) {
      // Inter-process fallback.
      inf_ptr->enqueue(holder.cpu_mat,
                       cv::Rect(0, 0, holder.cpu_mat.cols, holder.cpu_mat.rows));
    } else {
      continue;
    }
    inf_ptr->submitRequest();
    fprintf(stderr, "[VaPipeline] submitRequest OK\n");
  }

  // Feed cpu_mat to outputs for annotation / display.
  if (!holder.cpu_mat.empty()) {
    for (auto& [name, out_ptr] : getOutputHandle()) {
      out_ptr->feedFrame(holder.cpu_mat);
    }
  }

  // Wait, collect results, route to outputs.
  for (auto& [name, inf_ptr] : va_inferences_) {
    fprintf(stderr, "[VaPipeline] fetchResults start\n");
    inf_ptr->fetchResults();
    fprintf(stderr, "[VaPipeline] fetchResults done\n");
    // observeOutput calls handleOutput() which requires a CPU BGR frame
    // (for annotation / RViz image publishing).  Skip when there is no
    // cpu_mat — i.e. pure VA surface path with no CPU decode.
    if (!holder.cpu_mat.empty()) {
      for (auto& [oname, out_ptr] : getOutputHandle()) {
        fprintf(stderr, "[VaPipeline] observeOutput %s\n", oname.c_str());
        inf_ptr->observeOutput(out_ptr);
        fprintf(stderr, "[VaPipeline] observeOutput %s done\n", oname.c_str());
      }
    }
  }
}
