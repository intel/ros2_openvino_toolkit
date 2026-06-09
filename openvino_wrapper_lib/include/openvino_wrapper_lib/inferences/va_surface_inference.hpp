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

/**
 * @brief BaseInference subclass that feeds a VASurfaceID directly into OV
 *        via VAContext::create_tensor_nv12() — zero copy on iGPU.
 *
 * Wraps any existing inference type (ObjectDetection, etc.) and intercepts
 * the enqueue step.  All other pipeline slots (fetchResults, observeOutput,
 * getResults) delegate to the wrapped inference so output handling is
 * unchanged.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <va/va.h>
#include <openvino/openvino.hpp>
#include <openvino/runtime/intel_gpu/ocl/va.hpp>

#include "openvino_wrapper_lib/inferences/base_inference.hpp"
#include "openvino_wrapper_lib/inputs/va_surface_holder.hpp"

namespace openvino_wrapper_lib
{

/**
 * @class VaSurfaceInference
 * @brief Thin wrapper around a BaseInference that provides a VA-surface enqueue
 *        path alongside the normal cv::Mat path.
 *
 * VaPipeline calls enqueueVaSurface(); the base Pipeline fallback still calls
 * enqueue(cv::Mat) which delegates to the wrapped inference normally.
 */
class VaSurfaceInference : public BaseInference
{
public:
  explicit VaSurfaceInference(std::shared_ptr<BaseInference> wrapped)
    : wrapped_(std::move(wrapped))
  {
    if (wrapped_) setMaxBatchSize(wrapped_->getMaxBatchSize());
  }

  // ── VA surface path ────────────────────────────────────────────────────────

  /**
   * @brief Feed a VASurface directly into the InferRequest without any copy.
   *
   * Requires the Engine to have been created via EngineManager::createVaEngine()
   * (compiled against VAContext with PPP memory_type = GPU_SURFACE).
   *
   * @param holder   VaSurfaceHolder with valid va_surface_id and va_display.
   * @return         true on success.
   */
  bool enqueueVaSurface(const VaSurfaceHolder& holder)
  {
    if (!holder.hasVaSurface()) return false;
    auto eng = getEngine();
    if (!eng) return false;

    // Recover the VAContext from the compiled model's remote context.
    auto va_ctx = eng->getCompiledModel()
                      .get_context()
                      .as<ov::intel_gpu::ocl::VAContext>();

    // create_tensor_nv12 returns (y_tensor, uv_tensor) — zero-copy wrappers
    // over the VA surface's OpenCL image planes.
    auto [y_t, uv_t] = va_ctx.create_tensor_nv12(
        holder.height, holder.width,
        static_cast<VASurfaceID>(holder.va_surface_id));

    ov::InferRequest& req = eng->getRequest();
    req.set_input_tensor(0, y_t);
    req.set_input_tensor(1, uv_t);

    enqueued_frames_ = 1;
    return true;
  }

  // ── BaseInference delegation ───────────────────────────────────────────────

  bool enqueue(const cv::Mat& frame, const cv::Rect& loc) override
  {
    return wrapped_ ? wrapped_->enqueue(frame, loc) : false;
  }

  bool submitRequest() override
  {
    // For the VA path the engine is already set on this object; submitRequest
    // calls engine_->getRequest().start_async() via the base class.
    return BaseInference::submitRequest();
  }

  bool fetchResults() override
  {
    // Wait on our own request, then delegate result parsing to wrapped_.
    bool ok = BaseInference::fetchResults();
    if (!ok || !wrapped_) return ok;

    // Copy the output tensor from our engine into the wrapped inference engine
    // so its fetchResults() can parse it.
    if (getEngine() && wrapped_->getEngine()) {
      auto out = getEngine()->getRequest().get_output_tensor();
      wrapped_->getEngine()->getRequest().set_output_tensor(out);
    }
    return wrapped_->fetchResults();
  }

  void observeOutput(const std::shared_ptr<Outputs::BaseOutput>& output) override
  {
    if (wrapped_) wrapped_->observeOutput(output);
  }

  int getResultsLength() const override
  {
    return wrapped_ ? wrapped_->getResultsLength() : 0;
  }

  const Result* getLocationResult(int idx) const override
  {
    return wrapped_ ? wrapped_->getLocationResult(idx) : nullptr;
  }

  const std::string getName() const override
  {
    return wrapped_ ? (wrapped_->getName() + " [VA]") : "VaSurfaceInference";
  }

  const std::vector<cv::Rect> getFilteredROIs(const std::string filter) const override
  {
    return wrapped_ ? wrapped_->getFilteredROIs(filter) : std::vector<cv::Rect>{};
  }

private:
  std::shared_ptr<BaseInference> wrapped_;
};

}  // namespace openvino_wrapper_lib
