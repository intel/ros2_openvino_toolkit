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
 * @brief A header file with declaration for StandardCamera class
 * @file standard_camera.h
 */

#ifndef OPENVINO_WRAPPER_LIB__INPUTS__STANDARD_CAMERA_HPP_
#define OPENVINO_WRAPPER_LIB__INPUTS__STANDARD_CAMERA_HPP_

#include <opencv2/opencv.hpp>

#include "openvino_wrapper_lib/inputs/base_input.hpp"
#include <stdio.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace Input
{
/**
 * @class StandardCamera
 * @brief Class for recieving a standard camera as input.
 */
class StandardCamera : public BaseInputDevice
{
public:
  StandardCamera(const std::string& camera = "");
  /**
   * @brief Initialize the input device,
   * for cameras, it will turn the camera on and get ready to read frames,
   * for videos, it will open a video file.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize() override;
  /**
   * @brief Initialize the input device with given width and height.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize(size_t width, size_t height) override;
  /**
   * @brief Read next frame, and give the value to argument frame.
   * @return Whether the next frame is successfully read.
   */
  bool read(cv::Mat* frame) override;

private:
  int getCameraId();
  cv::VideoCapture cap;
  int camera_id_ = -1;
  std::string device_path_;
};
}  // namespace Input
#endif  // OPENVINO_WRAPPER_LIB__INPUTS__STANDARD_CAMERA_HPP_
