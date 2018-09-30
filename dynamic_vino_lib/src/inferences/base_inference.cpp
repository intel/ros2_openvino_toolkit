/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief a header file with declaration of BaseInference class
 * @file base_inference.cpp
 */

#include <memory>

#include "dynamic_vino_lib/inferences/base_inference.hpp"

// Result
dynamic_vino_lib::Result::Result(const cv::Rect& location) {
  location_ = location;
}

// BaseInference
dynamic_vino_lib::BaseInference::BaseInference() = default;

dynamic_vino_lib::BaseInference::~BaseInference() = default;

void dynamic_vino_lib::BaseInference::loadEngine(
    const std::shared_ptr<Engines::Engine> engine) {
  engine_ = engine;
}

bool dynamic_vino_lib::BaseInference::submitRequest() {
  if (engine_->getRequest() == nullptr) return false;
  if (!enqueued_frames) return false;
  enqueued_frames = 0;
  results_fetched_ = false;
  engine_->getRequest()->StartAsync();
  return true;
}

bool dynamic_vino_lib::BaseInference::fetchResults() {
  if (results_fetched_) return false;
  results_fetched_ = true;
  return true;
}
