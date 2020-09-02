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
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/models/attributes/base_attribute.hpp"

// Validated Base Network
Models::BaseModel::BaseModel(
  const std::string & model_loc, int max_batch_size)
: model_loc_(model_loc),
  max_batch_size_(max_batch_size)
{
  if (model_loc.empty()) {
    throw std::logic_error("model file name is empty!");
  }
  attr_->setModelName(model_loc);

  net_reader_ = std::make_shared<InferenceEngine::CNNNetReader>();
}

void Models::BaseModel::modelInit()
{
  slog::info << "Loading network files" << slog::endl;
  // Read network model
  net_reader_->ReadNetwork(model_loc_);
  // Set batch size to given max_batch_size_
  slog::info << "Batch size is set to  " << max_batch_size_ << slog::endl;
  net_reader_->getNetwork().setBatchSize(max_batch_size_);
  // Extract model name and load it's weights
  // remove extension
  size_t last_index = model_loc_.find_last_of(".");
  std::string raw_name = model_loc_.substr(0, last_index);
  std::string bin_file_name = raw_name + ".bin";
  net_reader_->ReadWeights(bin_file_name);
  // Read labels (if any)
  std::string label_file_name = raw_name + ".labels";
  attr_->loadLabelsFromFile(label_file_name);

  /** DEPRECATED!
  checkLayerProperty(net_reader_);
  setLayerProperty(net_reader_); */
  updateLayerProperty(net_reader_);
}

void Models::BaseModel::updateLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
  for(auto attr : candidated_attrs_) {
    if (attr == nullptr) continue;
    if (attr->updateLayerProperty(net_reader)){
      if(attr_!=nullptr){
        slog::warn << "The old attribute instance is replaced for model " << attr_->getModelName()
          << ". Currently each model only supports one Attribute instance." << slog::endl;
      }
      attr_ = attr;
    }
  }
  if(attr_ == nullptr){
    slog::warn << "The attribute instance is set for model " << attr_->getModelName()
          << ", using default settings" << slog::endl;
    attr_ = std::make_shared<Models::ModelAttribute>("DefaultAttr");
  }
}

Models::ObjectDetectionModel::ObjectDetectionModel(
  const std::string & model_loc,
  int max_batch_size)
: BaseModel(model_loc, max_batch_size) {}
