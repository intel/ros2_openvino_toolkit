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
 * @brief A header file with declaration for RealSenseCamera class
 * @file realsense_camera.h
 */

#ifndef OPENVINO_WRAPPER_LIB__INPUTS__REALSENSE_CAMERA_HPP_
#define OPENVINO_WRAPPER_LIB__INPUTS__REALSENSE_CAMERA_HPP_

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "openvino_wrapper_lib/inputs/base_input.hpp"

namespace Input
{
/**
 * @class RealSenseCamera
 * @brief Class for recieving a realsense camera as input.
 */
class RealSenseCamera : public BaseInputDevice
{
public:
  RealSenseCamera() {};
  /**
   * @brief Initialize the input device, turn the
   * camera on and get ready to read frames.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize() override;
  /**
   * @brief Initialize the input device,
   * @brief Initialize the input device, turn the
   * camera on and get ready to read frames.
   * @return Whether the input device is successfully setup.
   */
  bool initialize(const std::string &ip_uri) override { return initialize();};
  /**
   * @brief Initialize the input device with given width and height.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize(size_t width, size_t height) override;
  /**
   * @brief Read next frame, and give the value to argument frame.
   * @return Whether the next frame is successfully read.
   */
  bool read(cv::Mat * frame) override;

private:
  void bypassFewFramesOnceInited();
  std::string getCameraSN();

  rs2::config cfg_;
  rs2::pipeline pipe_;
  bool first_read_ = true;
  static int rscamera_count;
};
}  // namespace Input

#endif  // OPENVINO_WRAPPER_LIB__INPUTS__REALSENSE_CAMERA_HPP_
