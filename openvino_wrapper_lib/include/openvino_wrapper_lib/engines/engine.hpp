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
 * @brief A header file with declaration for Inference Engine class
 * @file engine.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__ENGINES__ENGINE_HPP_
#define OPENVINO_WRAPPER_LIB__ENGINES__ENGINE_HPP_

#pragma once

#include <string>
#include "openvino_wrapper_lib/models/base_model.hpp"
#include "openvino/openvino.hpp"

namespace Engines
{
/**
 * @class Engine
 * @brief This class manages the instance created for computing Engine hardware
 * (CPU|GPU|MYRIAD|HETERO|more). Currently it mainly manages the inference request(s)
 * and callback function for the Engine instance.
 */
class Engine
{
public:
  /**
   * @brief Using an Inference Request to initialize the inference Engine.
   */
  Engine(ov::InferRequest&);
  /**
   * @brief Initialize the engine from an inference request, keeping the compiled model
   * and the requested device so that the actually used device(s) can be queried.
   */
  Engine(ov::InferRequest&, const ov::CompiledModel&, const std::string& requested_device);
  /**
   * @brief Get the device(s) this engine is executing on, e.g. "CPU", "GPU.0" or "NPU".
   * For virtual devices (AUTO/HETERO/MULTI) this reports the actual execution device(s).
   * Falls back to the requested device name if the plugin can't report it.
   */
  std::string getDeviceName() const;
  /**
   * @brief Get the inference request this instance holds.
   * @return The inference request this instance holds.
   */
  inline ov::InferRequest& getRequest()
  {
    return request_;
  }
  /**
   * @brief Set a callback function for the infer request.
   * @param[in] callbackToSet A lambda function as callback function.
   * The callback function will be called when request is finished.
   */
  template <typename T>
  void setCompletionCallback(const T& callbackToSet)
  {
    request_.set_callback(callbackToSet);
  }

private:
  ov::InferRequest request_;
  ov::CompiledModel compiled_model_;
  std::string requested_device_;
};
}  // namespace Engines

#endif  // OPENVINO_WRAPPER_LIB__ENGINES__ENGINE_HPP_
