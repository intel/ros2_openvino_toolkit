// Copyright (c) 2026 Intel Corporation
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
 * @brief A header file with declaration for OrientedObjectDetection Class
 * @file oriented_object_detection.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__ORIENTED_OBJECT_DETECTION_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__ORIENTED_OBJECT_DETECTION_HPP_
#include <string>
#include "openvino_wrapper_lib/inferences/model_inference.hpp"

namespace Models
{
class OrientedObjectDetectionYolo26Model;
}

namespace openvino_wrapper_lib
{
/**
 * @class OrientedObjectDetectionResult
 * @brief Class for storing a detected object with an oriented (rotated) bounding box.
 * The location of the result is the upright rectangle around the oriented box.
 */
class OrientedObjectDetectionResult : public Result
{
public:
  explicit OrientedObjectDetectionResult(const cv::RotatedRect& box) : Result(box.boundingRect()), box_(box)
  {
  }
  inline std::string getLabel() const
  {
    return label_;
  }
  inline void setLabel(const std::string& label)
  {
    label_ = label;
  }
  inline float getConfidence() const
  {
    return confidence_;
  }
  inline void setConfidence(float confidence)
  {
    confidence_ = confidence;
  }
  /**
   * @brief Get the oriented box in frame coordinates. Its angle is in degrees, clockwise.
   */
  inline const cv::RotatedRect& getBox() const
  {
    return box_;
  }

private:
  std::string label_ = "";
  float confidence_ = -1;
  cv::RotatedRect box_;
};

/**
 * @brief Inference that detects objects with oriented bounding boxes.
 */
using OrientedObjectDetection =
    ModelInference<Models::OrientedObjectDetectionYolo26Model, OrientedObjectDetectionResult>;
}  // namespace openvino_wrapper_lib
#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__ORIENTED_OBJECT_DETECTION_HPP_
