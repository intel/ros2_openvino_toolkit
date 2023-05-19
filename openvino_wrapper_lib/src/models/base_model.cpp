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
#include "openvino_wrapper_lib/engines/engine.hpp"

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

Models::ObjectDetectionModel::ObjectDetectionModel(
  const std::string& label_loc, 
  const std::string& model_loc,
  int max_batch_size)
: BaseModel(label_loc, model_loc, max_batch_size) {}

cv::Mat Models::ObjectDetectionModel::resizeImage(const cv::Mat &orig_image, const int resized_width, const int resized_height,
  double fx, double fy, int interpolation, int border_dt, int border_db, int border_dl, int border_dr, cv::Scalar border_color) {
    cv::Mat resized_image;
    if (resized_width != orig_image.size().width || resized_height != orig_image.size().height) {
      cv::resize(orig_image, resized_image, cv::Size(resized_width, resized_height), fx, fy, interpolation);
    }
    if ( border_dt != 0 || border_db != 0 || border_dl != 0 || border_dr != 0) {
      cv::copyMakeBorder(resized_image,
                         resized_image,
                         border_dt,
                         border_db,
                         border_dl,
                         border_dr,
                         cv::BORDER_CONSTANT,
                         border_color);
    }
    return resized_image;
}

void Models::ObjectDetectionModel::dataToBlob(cv::Mat& resize_img, float scale_factor, int batch_index,
  const std::shared_ptr<Engines::Engine> & engine, bool mat_steps) {
    std::string input_name = getInputName();
    slog::debug << "add input image to blob: " << input_name << slog::endl;
    ov::Tensor in_tensor = engine->getRequest().get_tensor(input_name);
    u_int8_t * blob_data = (u_int8_t *)in_tensor.data();

    size_t height = resize_img.rows;
    size_t width = resize_img.cols;
    auto *input_data = (float *) resize_img.data;

    ov::Shape in_shape = in_tensor.get_shape();
    long unsigned int channels;
    if (mat_steps) {
      channels = in_shape[1];
      int batchOffset = batch_index * width * height * channels;
      for (int c = 0; c < channels; c++) {
        for (int h = 0; h < height; h++) {
          for (int w = 0; w < width; w++) {
            blob_data[batchOffset + c * width * height + h * width + w] =
              resize_img.at<cv::Vec3b>(h, w)[c] * scale_factor;
          }
        }
      }
    } else {
      channels = in_shape[3];
      ov::Tensor input_tensor;
      input_tensor = ov::Tensor(ov::element::u8, {1, height, width, channels}, input_data);
      engine->getRequest().set_input_tensor(input_tensor);
    }
}
