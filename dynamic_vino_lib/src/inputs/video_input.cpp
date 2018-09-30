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
 * @brief a header file with declaration of Video class
 * @file video_input.cpp
 */

#include <string>

#include "dynamic_vino_lib/inputs/video_input.hpp"

// Video
Input::Video::Video(const std::string& video) { video_.assign(video); }

bool Input::Video::initialize() {
  setInitStatus(cap.open(video_));
  setWidth((size_t)cap.get(CV_CAP_PROP_FRAME_WIDTH));
  setHeight((size_t)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  return isInit();
}

bool Input::Video::initialize(size_t width, size_t height) {
  setWidth(width);
  setHeight(height);
  setInitStatus(cap.open(video_));
  if (isInit()) {
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  }
  return isInit();
}

bool Input::Video::read(cv::Mat* frame) {
  if (!isInit()) {
    return false;
  }
  cap.grab();
  return cap.retrieve(*frame);
}

void Input::Video::config() {
  // TODO(weizhi): config
}
