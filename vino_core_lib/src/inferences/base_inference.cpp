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
 * @brief a header file with declaration of BaseInference class
 * @file base_inference.cpp
 */

#include <memory>

#include "vino_core_lib/inferences/base_inference.hpp"
#include "vino_core_lib/models/base_model.hpp"

 // Result
vino_core_lib::Result::Result(const cv::Rect & location)
{
  location_ = location;
}

// BaseInference
vino_core_lib::BaseInference::BaseInference() = default;

vino_core_lib::BaseInference::~BaseInference() = default;

void vino_core_lib::BaseInference::loadEngine(const std::shared_ptr<Engines::Engine> engine)
{
  engine_ = engine;
}

bool vino_core_lib::BaseInference::submitRequest()
{
  if (engine_->getRequest() == nullptr) {
    return false;
  }
  if (!enqueued_frames_) {
    return false;
  }
  enqueued_frames_ = 0;
  results_fetched_ = false;
  engine_->getRequest()->StartAsync();
  slog::debug << "Async Inference started!" << slog::endl;
  return true;
}

bool vino_core_lib::BaseInference::SynchronousRequest()
{
  if (engine_->getRequest() == nullptr) {
    return false;
  }
  if (!enqueued_frames_) {
    return false;
  }
  enqueued_frames_ = 0;
  results_fetched_ = false;
  engine_->getRequest()->Infer();
  return true;
}

bool vino_core_lib::BaseInference::fetchResults()
{
  if (results_fetched_) {
    return false;
  }
  results_fetched_ = true;
  return true;
}

void vino_core_lib::BaseInference::addCandidatedModel(std::shared_ptr<Models::BaseModel> model)
{
  slog::info << "TESTING in addCandidatedModel()" << slog::endl;
  if (model != nullptr) {
    slog::info << "adding new Model Candidate..." << slog::endl;
    candidated_models_.push_back(model);
  }
}
