// Copyright (c) 2018 Intel Corporation
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
#ifndef DYNAMIC_VINO_LIB__ENGINES__ENGINE_HPP_
#define DYNAMIC_VINO_LIB__ENGINES__ENGINE_HPP_

#pragma once

#include "dynamic_vino_lib/models/base_model.hpp"
#include "inference_engine.hpp"

namespace Engines
{
/**
 * @class Engine
 * @brief This class manages the instance created for computing Engine hardware
 * (CPU|GPU|MARIAD|HETERO|more). Currently it mainly manages the inference request(s)
 * and callback function for the Engine instance.
 */
class Engine
{
public:
#if(defined(USE_OLD_E_PLUGIN_API))
  /**
   * DEPRECATED! instead of using Engine(InferenceEngine::InferRequest::Ptr &)
   * @brief Create an NetworkEngine instance
   * from a inference plugin and an inference network.
   */
  Engine(InferenceEngine::InferencePlugin, Models::BaseModel::Ptr);
#endif

  /**
   * @brief Using an Inference Request to initialize the inference Engine.
   */
  Engine(InferenceEngine::InferRequest::Ptr &);
  Engine(const std::string &);
  /**
   * @brief Get the inference request this instance holds.
   * @return The inference request this instance holds.
   */
  inline InferenceEngine::InferRequest::Ptr & getRequest()
  {
    return request_;
  }
  /**
   * @brief Set a callback function for the infer request.
   * @param[in] callbackToSet A lambda function as callback function.
   * The callback function will be called when request is finished.
   */
  template<typename T>
  void setCompletionCallback(const T & callbackToSet)
  {
    request_->SetCompletionCallback(callbackToSet);
  }

  InferenceEngine::CNNNetwork & prepareNetwork(const std::string, const int);
  InferenceEngine::CNNNetwork * getNetwork(void);

private:
  InferenceEngine::InferRequest::Ptr request_ = nullptr;
  InferenceEngine::Core ie_;
  InferenceEngine::CNNNetwork network_;
  InferenceEngine::ExecutableNetwork executable_network_;
  std::string device_;
};
}  // namespace Engines

#endif  // DYNAMIC_VINO_LIB__ENGINES__ENGINE_HPP_
