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
 * @brief a header file with declaration of Image class
 * @file image_input.cpp
 */

#include <string>
#include "dynamic_vino_lib/inputs/image_input.hpp"

Input::Image::Image(const std::string& file) { file_.assign(file); }

bool Input::Image::initialize() {
  image_ = cv::imread(file_);
  if (image_.data != NULL) {
    setInitStatus(true);
    setWidth((size_t)image_.cols);
    setHeight((size_t)image_.rows);
  } else {
    setInitStatus(false);
  }
  return isInit();
}

bool Input::Image::read(cv::Mat* frame) {
  if (!isInit()) {
    return false;
  }
  *frame = image_;
  return true;
}

void Input::Image::config() {
  // TODO(weizhi): config
}