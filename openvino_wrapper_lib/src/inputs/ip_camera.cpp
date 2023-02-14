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
 * @brief a header file with declaration of IpCamera class
 * @file ip_camera.cpp
 */
#include "openvino_wrapper_lib/inputs/ip_camera.hpp"


bool Input::IpCamera::initialize()
{
  // Initialize width and height to reasonable dimensions
  return initialize(640, 480);
}

bool Input::IpCamera::initialize(size_t width, size_t height)
{
  setInitStatus(cap.open(ip_uri_));
  if (isInit()) {
    setWidth(width);
    setHeight(height);
  }
  return isInit();
}

bool Input::IpCamera::read(cv::Mat * frame)
{
  if (!isInit()) {
    return false;
  }
  cap.grab();
  setHeader("ip_camera_frame");
  bool retrieved = cap.retrieve(*frame);
  if (retrieved) {
    cv::resize(*frame, *frame, cv::Size(getWidth(), getHeight()), 0, 0, CV_INTER_AREA);
  }
  return retrieved;
}
