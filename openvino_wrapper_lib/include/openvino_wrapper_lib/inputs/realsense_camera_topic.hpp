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

#ifndef OPENVINO_WRAPPER_LIB__INPUTS__REALSENSE_CAMERA_TOPIC_HPP_
#define OPENVINO_WRAPPER_LIB__INPUTS__REALSENSE_CAMERA_TOPIC_HPP_

#include "openvino_wrapper_lib/inputs/image_topic.hpp"

namespace Input
{
/**
 * DEPRECATED!
 * Using the new class ImageTopic to handle all image topics.
 * @class RealSenseCameraTopic
 * @brief Class for recieving a realsense camera topic as input.
 */
typedef ImageTopic RealSenseCameraTopic;

}  // namespace Input

#endif  // OPENVINO_WRAPPER_LIB__INPUTS__REALSENSE_CAMERA_TOPIC_HPP_
