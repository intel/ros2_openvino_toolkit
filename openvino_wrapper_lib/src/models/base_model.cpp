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
#include "openvino_wrapper_lib/utils/common.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/models/attributes/base_attribute.hpp"
#include "openvino_wrapper_lib/models/attributes/base_attribute.hpp"

// Validated Base Network
Models::BaseModel::BaseModel(
  const std::string& label_loc, const std::string& model_loc, int max_batch_size)
: label_loc_(label_loc),
  model_loc_(model_loc),
  max_batch_size_(max_batch_size),
  ModelAttribute(model_loc)
{
  if (model_loc.empty()) {
    throw std::logic_error("model file name is empty!");
  }

}

void Models::BaseModel::modelInit()
{
  slog::info << "Loading network files" << model_loc_ << slog::endl;
  slog::info << label_loc_ << slog::endl;
  
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

Models::ObjectDetectionModel::ObjectDetectionModel(
  const std::string& label_loc, 
  const std::string& model_loc,
  int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

bool Models::BaseModel::matToBlob(
  const cv::Mat & orig_image, const cv::Rect &, float scale_factor,
  int batch_index, const std::shared_ptr<Engines::Engine> & engine)
{
  if (engine == nullptr)
  {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  ov::InferRequest infer_request = engine->getRequest();
  ov::Tensor input_tensor = infer_request.get_tensor(getInputName("input0"));
  ov::Shape input_shape = input_tensor.get_shape();

  OPENVINO_ASSERT(input_shape.size() == 4);
  const auto layout = getLayoutFromShape(input_shape);
  const size_t width = input_shape[ov::layout::width_idx(layout)]; //input_shape[2];
  const size_t height = input_shape[ov::layout::height_idx(layout)]; //input_shape[1];
  const size_t channels = input_shape[ov::layout::channels_idx(layout)]; //input_shape[3];

  slog::debug <<"width is:"<< width << slog::endl;
  slog::debug <<"height is:"<< height << slog::endl;
  slog::debug <<"channels is:"<< channels << slog::endl;
  slog::debug <<"origin channels is:"<< orig_image.channels() << slog::endl;
  slog::debug <<"input shape is:"<< input_shape << slog::endl;

  unsigned char* data = input_tensor.data<unsigned char>();
  cv::Size size = {(int)width, (int)height};
  cv::Mat resized_image(size, CV_8UC3, data);

  if ( isKeepInputRatio()){
    slog::debug << "keep Input Shape Ratio is ENABLED!" << slog::endl;
    cv::Mat extend_image = extendFrameToInputRatio(orig_image);
    cv::resize(extend_image, resized_image, size);
    frame_resize_ratio_width_ = static_cast<float>(extend_image.cols) / width;
    frame_resize_ratio_height_ = static_cast<float>(extend_image.rows) / height;
  } else {
    cv::resize(orig_image, resized_image, size);
    frame_resize_ratio_width_ = static_cast<float>(orig_image.cols) / width;
    frame_resize_ratio_height_ = static_cast<float>(orig_image.rows) / height;
  }

  return true;
}

cv::Mat Models::BaseModel::extendFrameToInputRatio(const cv::Mat orig)
{
  auto orig_width = orig.cols;
  auto orig_height = orig.rows;
  const auto target_width = getInputWidth();
  const auto target_height = getInputHeight();
  const float orig_ratio = static_cast<float>(orig_width) / orig_height;
  const float target_ratio = static_cast<float>(target_width) / target_height;

  slog::debug << "extend Ratio: orit_ratio:"<< orig_ratio << ", target_ratio:" << target_ratio <<
    ", orig_width:" << orig_width << ", orig_height:" << orig_height << slog::endl;
  if (orig_ratio < target_ratio){
    orig_width = (int)(orig_height * target_ratio);
  }else{
    orig_height = (int)(orig_width * target_ratio);
  }

  slog::debug << "extend Image to: " << orig_width << "x" << orig_height << slog::endl;
  cv::Mat result = cv::Mat::zeros(orig_height, orig_width, CV_8UC3);
  orig.copyTo(result(cv::Rect(0, 0, orig.cols, orig.rows)));

  return result;
}

bool Models::BaseModel::updateLayerProperty(
    std::shared_ptr<ov::Model>& model)
{
  slog::info<< "Checking INPUTS & OUTPUTS for Model " <<getModelName()<<slog::endl;

  // check input shape
  inputs_info_ = model->inputs();
  slog::debug <<"input size="<<inputs_info_.size()<<slog::endl;
  if (inputs_info_.size() != getCountOfInputs() ) {
    slog::warn << "This inference sample should have have " << getCountOfInputs()
      << " inputs, but we got" << std::to_string(inputs_info_.size()) << "inputs"
      << slog::endl;
    throw std::logic_error("input_tensor_count doesn't align!");
    return false;
  }

  for (int i = 0; i < getCountOfInputs(); i++){
    std::string name{"input"};
    addInputInfo(name+std::to_string(i), inputs_info_[i].get_any_name());
  }

  // check output shape
  outputs_info_ = model->outputs();
  slog::debug <<"output size="<<outputs_info_.size()<<slog::endl;
  if (outputs_info_.size() != getCountOfOutputs()) {
    slog::warn << "This inference sample should have have " << getCountOfOutputs()
      <<" outputs, but we got " <<outputs_info_.size() << "outputs"
      << slog::endl;
    throw std::logic_error("output_tensor_count doesn't align!");
    return false;
  }

  for (int i = 0; i < getCountOfOutputs(); i++){
    std::string name{"output"};
    addOutputInfo(name+std::to_string(i), outputs_info_[i].get_any_name());
  }

  return true;
}
