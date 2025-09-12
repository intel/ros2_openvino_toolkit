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
 * @brief a header file with declaration of BaseModel class
 * @file base_model.cpp
 */

#include <fstream>
#include <string>
#include <memory>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include "openvino_wrapper_lib/models/base_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/models/attributes/base_attribute.hpp"

// Validated Base Network
void Models::BaseModel::modelInit(const Params::ParamManager::InferenceRawData & param, bool use_def_batch, int batch_size)
{
  if (param.model.empty()) {
    throw std::logic_error("model file name is empty!");
  }

  label_loc_ = param.label;
  model_loc_ = param.model;
  attr_.model_name = param.model;
  max_batch_size_ = use_def_batch ? batch_size : param.batch;

  slog::info << "Loading network files: " << model_loc_ << slog::endl;
  
  // Read network model
  model_ = engine.read_model(model_loc_);
  
  // Extract model name and load it's weights
  // remove extension
  size_t last_index = model_loc_.find_last_of(".");
  std::string raw_name = model_loc_.substr(0, last_index);
  
  // Read labels (if any)
  std::string label_file_name = label_loc_.substr(0, last_index);
  loadLabelsFromFile(label_loc_);

  // Set batch size to given max_batch_size_
  slog::info << "Batch size is set to  " << max_batch_size_ << slog::endl;
  updateLayerProperty(model_);
}

#if 0
bool Models::BaseModel::updateLayerProperty(
  InferenceEngine::CNNNetReader::Ptr model)
{
#if 0
  if (!updateLayerProperty(model)){
    slog::warn << "The model(name: " << getModelName() << ") failed to update Layer Property!"
      << slog::endl;
    return false;
  }
#endif
  if(!isVerified()){
    slog::warn << "The model(name: " << getModelName() << ") does NOT pass Attribute Check!"
      << slog::endl;
    return false;
  }

  return true;
}
#endif
