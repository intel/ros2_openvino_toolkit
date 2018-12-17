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
 * @brief a header file with declaration of BaseModel class
 * @file base_model.cpp
 */

#include <fstream>
#include <string>
#include <memory>
#include <algorithm>

#include "dynamic_vino_lib/models/base_model.hpp"
#include "dynamic_vino_lib/slog.hpp"

// Validated Base Network
Models::BaseModel::BaseModel(const std::string& model_loc, int input_num,
                             int output_num, int max_batch_size)
    : input_num_(input_num),
      output_num_(output_num),
      model_loc_(model_loc),
      max_batch_size_(max_batch_size) {
  if (model_loc.empty()) {
    throw std::logic_error("model file name is empty!");
  }
  net_reader_ = std::make_shared<InferenceEngine::CNNNetReader>();
}

void Models::BaseModel::modelInit() {
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
  std::ifstream input_file(label_file_name);
  std::copy(std::istream_iterator<std::string>(input_file),
            std::istream_iterator<std::string>(), std::back_inserter(labels_));
  checkNetworkSize(input_num_, output_num_, net_reader_);
  checkLayerProperty(net_reader_);
  setLayerProperty(net_reader_);
}


void Models::BaseModel::checkNetworkSize(
    int input_size, int output_size,
    InferenceEngine::CNNNetReader::Ptr net_reader) {
  // TODO(Houk): Repeat, better removed!
  // check input size
  slog::info << "Checking input size" << slog::endl;
  InferenceEngine::InputsDataMap input_info(
      net_reader->getNetwork().getInputsInfo());
  if (input_info.size() != input_size) {
    throw std::logic_error(getModelName() + " should have only one input");
  }
  // check output size
  slog::info << "Checking output size" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(
      net_reader->getNetwork().getOutputsInfo());
  if (output_info.size() != output_size) {
    throw std::logic_error(getModelName() +
                           "network should have only one output");
  }
  // InferenceEngine::DataPtr& output_data_ptr = output_info.begin()->second;
}
