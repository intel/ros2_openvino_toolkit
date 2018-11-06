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
 * @brief A header file with declaration for FaceDetection Class
 * @file face_detection.h
 */
#ifndef DYNAMIC_VINO_LIB__INFERENCES__FACE_DETECTION_HPP_
#define DYNAMIC_VINO_LIB__INFERENCES__FACE_DETECTION_HPP_

#include <object_msgs/msg/object.hpp>
#include <object_msgs/msg/object_in_box.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>
#include <string>

#include "dynamic_vino_lib/models/face_detection_model.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/inferences/object_detection.hpp"
#include "inference_engine.hpp"
#include "opencv2/opencv.hpp"

// namespace
namespace dynamic_vino_lib {
/**
 * @class FaceDetectionResult
 * @brief Class for storing and processing face detection result.
 */
class FaceDetectionResult : public ObjectDetectionResult {
 public:
  explicit FaceDetectionResult(const cv::Rect& location);
};

/**
 * @class FaceDetection
 * @brief Class to load face detection model and perform face detection.
 */
class FaceDetection : public ObjectDetection {
 public:
  explicit FaceDetection(double);
};
}  // namespace dynamic_vino_lib
#endif  // DYNAMIC_VINO_LIB__INFERENCES__FACE_DETECTION_HPP_
