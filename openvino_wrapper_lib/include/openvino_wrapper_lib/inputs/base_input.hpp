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
 * @brief A header file with declaration for BaseInput Class
 * @file base_input.h
 */
#ifndef OPENVINO_WRAPPER_LIB__INPUTS__BASE_INPUT_HPP_
#define OPENVINO_WRAPPER_LIB__INPUTS__BASE_INPUT_HPP_

#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/videoio/videoio_c.h>
#include <vector>
#include <string>
#include "openvino_wrapper_lib/inputs/ros2_handler.hpp"

/**
 * @class BaseInputDevice
 * @brief This class is an interface for three kinds of
 * input devices: realsense camera, standard camera and video
 */
namespace Input
{
struct Config
{
  /** a file path related to Input Device. */
  std::string path;
};

class BaseInputDevice : public Ros2Handler
{
public:
  BaseInputDevice() = default;
  /**
   * @brief Initialize the input device,
   * for cameras, it will turn the camera on and get ready to read frames,
   * for videos, it will open a video file.
   * @return Whether the input device is successfully turned on.
   */
  virtual bool initialize() = 0;
  /**
   * @brief Initialize the input device,
   * @return Whether the input device is successfully setup.
   */
  virtual bool initialize(const std::string &str) = 0;
  /**
   * @brief Initialize the input device with given width and height.
   * @return Whether the input device is successfully turned on.
   */
  virtual bool initialize(size_t width, size_t height) = 0;
  /**
   * @brief Read next frame, and give the value to argument frame.
   * @return Whether the next frame is successfully read.
   */
  virtual bool read(cv::Mat * frame) = 0;
  virtual bool readService(cv::Mat * frame, std::string config_path)
  {
    return true;
  }
  virtual void config(const Config &) {}
  virtual ~BaseInputDevice() = default;
  /**
   * @brief Get the width of the frame read from input device.
   * @return The width of the frame read from input device.
   */
  inline size_t getWidth()
  {
    return width_;
  }
  /**
   * @brief Set the width of the frame read from input device.
   * @param[in] width Width to be set for the frame.
   */
  inline void setWidth(size_t width)
  {
    width_ = width;
  }
  /**
   * @brief Get the height of the frame read from input device.
   * @return The height of the frame read from input device.
   */
  inline size_t getHeight()
  {
    return height_;
  }
  /**
   * @brief Set the height of the frame read from input device.
   * @param[in] width Width to be set for the frame.
   */
  inline void setHeight(size_t height)
  {
    height_ = height;
  }
  /**
   * @brief Check whether the input device is successfully initiated.
   * @return Whether the input device is successfully initiated.
   */
  inline bool isInit()
  {
    return is_init_;
  }
  /**
   * @brief Set the initialization state for input device.
   * @param[in] is_init The initialization state to be set.
   */
  inline void setInitStatus(bool is_init)
  {
    is_init_ = is_init;
  }

private:
  size_t width_ = 0;  // 0 means using the original size
  size_t height_ = 0; // 0 means using the original size
  bool is_init_ = false;
};
}  // namespace Input
#endif  // OPENVINO_WRAPPER_LIB__INPUTS__BASE_INPUT_HPP_
