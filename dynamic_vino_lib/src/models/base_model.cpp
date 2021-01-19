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
 * @brief a header file with declaration of BaseModel class
 * @file base_model.cpp
 */

#include <fstream>
#include <string>
#include <memory>
#include <algorithm>
#include <iostream>
#include "dynamic_vino_lib/models/base_model.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/models/attributes/base_attribute.hpp"

// Validated Base Network
Models::BaseModel::BaseModel(
  const std::string & model_loc, int max_batch_size)
: model_loc_(model_loc),
  max_batch_size_(max_batch_size),
  ModelAttribute(model_loc)
{
  if (model_loc.empty()) {
    throw std::logic_error("model file name is empty!");
  }

  size_t last_index = model_loc_.find_last_of(".");
  std::string raw_name = model_loc_.substr(0, last_index);
  std::string bin_file_name = raw_name + ".bin";
  net_reader_->ReadWeights(bin_file_name);
  // Read labels (if any)
  std::string label_file_name = raw_name + ".labels";
  loadLabelsFromFile(label_file_name);
 }

void Models::BaseModel::modelInit(const std::shared_ptr<Engines::Engine> & engine)
{
  slog::info << "Loading network files" << slog::endl;
  if (engine == nullptr){
    slog::warn << "Engine instance should be assigned when init Module!"
      << slog::endl;
    return;
  }
  engine_ = engine;

  // Read network model
  network_ = engine->ReadNetwork(model_loc_);
  updateLayerProperty();
  // Set batch size to given max_batch_size_
  slog::info << "Batch size is set to  " << max_batch_size_ << slog::endl;
  net_reader_->getNetwork().setBatchSize(max_batch_size_);

  updateLayerProperty(net_reader_);
}

#if 0
bool Models::BaseModel::updateLayerProperty()
{
#if 0
  if (!updateLayerProperty()){
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

Models::ObjectDetectionModel::ObjectDetectionModel(
  const std::string & model_loc,
  int max_batch_size)
: BaseModel(model_loc, max_batch_size) {}
