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
 * @brief A header file with declaration for Image class
 * @file file_input.h
 */
#ifndef OPENVINO_WRAPPER_LIB__INPUTS__IMAGE_INPUT_HPP_
#define OPENVINO_WRAPPER_LIB__INPUTS__IMAGE_INPUT_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include "openvino_wrapper_lib/inputs/base_input.hpp"

namespace Input
{
/**
 * @class Image
 * @brief Class for recieving an image file as input.
 */
class Image : public BaseInputDevice
{
public:
  Image() {};
  explicit Image(const std::string &);
  /**
   * @brief Read an image file from the file path.
   * @param[in] An image file path.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize() override;

  /**
   * @brief Read an image file from the file path.
   * @param[in] An image file path.
   * @return Whether the input device is successfully setup.
   */
  bool initialize(const std::string &file) override {file_.assign(file); return initialize();};

  /**
   * @brief Initialize the input device with given width and height.
   * No implementation for Image class.
   * @return Whether the input device is successfully turned on.
   */
  bool initialize(size_t width, size_t height) override
  {
    return initialize();
  }
  /**
   * @brief Read next frame, and give the value to argument frame.
   * @return Whether the next frame is successfully read.
   */
  bool read(cv::Mat * frame) override;

  void config(const Config &) override;

private:
  cv::Mat image_;
  std::string file_;
};
}  // namespace Input

#endif  // OPENVINO_WRAPPER_LIB__INPUTS__IMAGE_INPUT_HPP_
