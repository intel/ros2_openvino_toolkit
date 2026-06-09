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
 * @brief Pipeline subclass that uses VaSurfaceTopic as input and routes each
 *        frame through VaSurfaceInference::enqueueVaSurface() for zero-copy
 *        GPU inference.
 *
 * Build a VaPipeline the same way as Pipeline, but:
 *   - Pass VaSurfaceTopic as the input device.
 *   - Wrap each ObjectDetection/inference in VaSurfaceInference.
 *   - Create the engine with EngineManager::createVaEngine() (GPU_SURFACE PPP).
 *
 * Intra-process contract:
 *   Both icamera_usm and this node must run in the same
 *   rclcpp::executors::MultiThreadedExecutor / ComponentManager with
 *   use_intra_process_comms=true.  The VaSurfaceFrame::va_surface_id field
 *   carries a live VASurfaceID valid in the shared VADisplay.
 *
 * Type negotiation:
 *   VaPipeline calls VaSurfaceTopic::publishRequirements() after the engine
 *   is compiled so the camera VEBOX/SFC scaler reconfigures its output to
 *   match the model's input resolution before the first inference frame.
 */
#pragma once

#include <map>
#include <memory>
#include <string>

#include "openvino_wrapper_lib/pipeline.hpp"
#include "openvino_wrapper_lib/inputs/va_surface_topic.hpp"
#include "openvino_wrapper_lib/inferences/va_surface_inference.hpp"
#include "openvino_wrapper_lib/engines/engine_manager.hpp"

class VaPipeline : public Pipeline
{
public:
  explicit VaPipeline(const std::string& name = "va_pipeline");

  /**
   * @brief Register a VaSurfaceInference with the pipeline.
   *
   * Call this (and Pipeline::add()) for each inference that should use the
   * VA surface path.  VaPipeline::runOnce() iterates va_inferences_ directly
   * so it does not need access to Pipeline's private name_to_detection_map_.
   *
   * @param name   Name used in Pipeline::add() calls.
   * @param inf    VaSurfaceInference (or subclass) instance.
   */
  void addVaInference(const std::string& name,
                      std::shared_ptr<openvino_wrapper_lib::VaSurfaceInference> inf);

  /**
   * @brief Override runOnce() to use the VASurface zero-copy path.
   *
   * When a VA surface is available from the input device, all
   * VaSurfaceInference instances in the pipeline receive it via
   * enqueueVaSurface().  If no VA surface is ready the call is a no-op
   * (frames are dropped rather than blocking).
   */
  void runOnce();

  /**
   * @brief Compile all registered VaSurfaceInference engines with
   *        EngineManager::createVaEngine() and publish InferRequirements.
   *
   * Call once after add()ing all inferences and after modelInit() has been
   * called on each model so that input dimensions are known.
   *
   * @param device   OV device string, e.g. "GPU".
   * @param va_dpy   Process-global VADisplay (from VaDisplayHolder::get()).
   */
  void compileVaEngines(const std::string& device, VADisplay va_dpy);

private:
  Input::VaSurfaceTopic* vaInput() const;

  // VA-aware inferences registered via addVaInference().
  std::map<std::string, std::shared_ptr<openvino_wrapper_lib::VaSurfaceInference>>
      va_inferences_;
};
