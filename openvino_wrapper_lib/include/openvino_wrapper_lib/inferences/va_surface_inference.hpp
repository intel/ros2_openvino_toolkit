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
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <va/va.h>
#include <openvino/openvino.hpp>
#include <openvino/runtime/intel_gpu/ocl/va.hpp>

#include "openvino_wrapper_lib/inferences/base_inference.hpp"
#include "openvino_wrapper_lib/inputs/va_surface_holder.hpp"
#include "va_display_holder.hpp"

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

  /// Set the expected VA surface dimensions (model input size).
  /// Frames that arrive at a different size are skipped during VEBOX reconfiguration.
  void setExpectedSize(uint32_t w, uint32_t h) { expected_w_ = w; expected_h_ = h; }
  uint32_t getExpectedWidth()  const { return expected_w_; }
  uint32_t getExpectedHeight() const { return expected_h_; }

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

    // Skip frames that haven't been scaled to the model's expected size yet.
    // After InferRequirements is published, the VEBOX reconfigures but a few
    // frames at the old resolution may still be in flight.
    if (expected_w_ > 0 && expected_h_ > 0) {
      if (holder.width != expected_w_ || holder.height != expected_h_) {
        return false;  // silent skip — VEBOX not yet reconfigured
      }
    }

    // Recover the VAContext from the compiled model's remote context.
    fprintf(stderr, "[VaSurfInf] get_context start\n");
    auto& compiled = eng->getCompiledModel();
    // Log input names once for diagnostics.
    static bool names_logged = false;
    if (!names_logged) {
      names_logged = true;
      for (size_t i = 0; i < compiled.inputs().size(); ++i)
        fprintf(stderr, "[VaSurfInf] compiled input[%zu] = \"%s\"\n",
                i, compiled.input(i).get_any_name().c_str());
    }
    auto va_ctx = compiled.get_context().as<ov::intel_gpu::ocl::VAContext>();
    fprintf(stderr, "[VaSurfInf] create_tensor_nv12 h=%u w=%u surf=%u\n",
            holder.height, holder.width, holder.va_surface_id);

    // Release previous VA surface tensors before acquiring new ones.
    // Each create_tensor_nv12 acquires the surface as a CL media interop object;
    // the driver may have a per-surface acquisition limit.
    last_y_t_  = ov::Tensor{};
    last_uv_t_ = ov::Tensor{};

    // Reuse cached tensors for each VASurfaceID — avoids repeated
    // clCreateFromVA_APIMediaSurfaceINTEL / clReleaseMemObject cycles which
    // exhaust a driver-side resource after ~285 calls.
    VASurfaceID surf_id = static_cast<VASurfaceID>(holder.va_surface_id);
    auto cache_it = surface_tensors_.find(surf_id);
    if (cache_it == surface_tensors_.end()) {
      auto [new_y, new_uv] = va_ctx.create_tensor_nv12(
          holder.height, holder.width, surf_id);
      surface_tensors_[surf_id] = {new_y, new_uv};
      cache_it = surface_tensors_.find(surf_id);
      fprintf(stderr, "[VaSurfInf] cached new tensor for surf=%u\n", surf_id);
    }
    auto& [y_t, uv_t] = cache_it->second;
    fprintf(stderr, "[VaSurfInf] create_tensor_nv12 OK (cached) — set_tensor\n");

    last_y_t_  = y_t;
    last_uv_t_ = uv_t;

    ov::InferRequest& req = eng->getRequest();
    // Bind via compiled model port objects — the only stable binding path for
    // VASurfaceTensors (string names and positional index are both unreliable).
    auto& cm = eng->getCompiledModel();
    req.set_tensor(cm.input(0), y_t);
    req.set_tensor(cm.input(1), uv_t);
    fprintf(stderr, "[VaSurfInf] set_input_tensor OK\n");

    enqueued_frames_ = 1;
    submitted_ = false;  // reset; set to true in submitRequest
    return true;
  }

  // ── BaseInference delegation ───────────────────────────────────────────────

  // Override loadEngine so the wrapped inference shares the VA engine.
  // This means wrapped_->fetchResults() → valid_model_->fetchResults(va_engine)
  // reads the output tensor from the completed VA inference request — correct.
  void loadEngine(const std::shared_ptr<Engines::Engine> engine) override
  {
    BaseInference::loadEngine(engine);
    if (wrapped_) wrapped_->loadEngine(engine);
    // Pre-build the surface→tensor cache once the engine (and thus the
    // compiled model with its VAContext) is known.
    surface_tensors_.clear();
  }

  bool enqueue(const cv::Mat& frame, const cv::Rect& loc) override
  {
    return wrapped_ ? wrapped_->enqueue(frame, loc) : false;
  }

  bool submitRequest() override
  {
    // Use synchronous infer() for the VA path so the GPU kernel completes
    // before returning — this ensures the VA surface isn't recycled by VEBOX
    // while the GPU is still reading it (no separate wait() needed).
    // Hold conversionMutex() during infer() to prevent VEBOX from touching
    // any VA surface concurrently with the iHD GPU kernel (driver-level
    // conflict on the shared VA display's OpenCL queue).
    auto eng = getEngine();
    if (!eng || !enqueued_frames_) return false;
    enqueued_frames_ = 0;
    results_fetched_ = false;
    fprintf(stderr, "[VaSurfInf] infer() start\n");
    {
      std::lock_guard<std::mutex> lk(icamera_usm::VaDisplayHolder::conversionMutex());
      eng->getRequest().infer();
    }
    fprintf(stderr, "[VaSurfInf] infer() done\n");
    submitted_ = true;
    if (wrapped_) wrapped_->resetFetchState();
    return true;
  }

  bool fetchResults() override
  {
    // Only wait if we actually submitted a request this round.
    // If enqueueVaSurface failed (e.g. wrong size during VEBOX reconfiguration)
    // submitted_ will be false and calling wait() on an un-started InferRequest
    // causes a SIGSEGV.
    if (!submitted_) return false;
    submitted_ = false;

    // infer() in submitRequest() already blocked until completion —
    // no need to wait() again.
    auto eng = getEngine();
    (void)eng;

    // BaseInference::fetchResults() flips results_fetched_.
    fprintf(stderr, "[VaSurfInf] BaseInference::fetchResults\n");
    bool ok = BaseInference::fetchResults();
    fprintf(stderr, "[VaSurfInf] BaseInference::fetchResults done ok=%d\n", (int)ok);
    if (!ok || !wrapped_) return ok;

    // The wrapped ObjectDetection shares the same VA engine (set by loadEngine
    // override above), so valid_model_->fetchResults(va_engine, ...) reads the
    // output tensor from the completed request — no tensor copy needed.
    fprintf(stderr, "[VaSurfInf] wrapped_->fetchResults\n");
    bool r = wrapped_->fetchResults();
    fprintf(stderr, "[VaSurfInf] wrapped_->fetchResults done r=%d\n", (int)r);
    return r;
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
  uint32_t expected_w_ = 0;
  uint32_t expected_h_ = 0;
  bool submitted_ = false;  // true only between submitRequest and fetchResults
  // Keep last-frame tensors alive until after infer() to prevent premature CL release.
  ov::Tensor last_y_t_;
  ov::Tensor last_uv_t_;
  // Cache VASurfaceID → (y_tensor, uv_tensor) so create_tensor_nv12 is only
  // called once per surface (avoids CL media-interop resource exhaustion).
  std::unordered_map<VASurfaceID, std::pair<ov::Tensor, ov::Tensor>> surface_tensors_;
};

}  // namespace openvino_wrapper_lib
