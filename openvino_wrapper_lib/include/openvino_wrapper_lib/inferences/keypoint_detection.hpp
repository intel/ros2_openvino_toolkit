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
 * @brief A header file with declaration for KeypointDetection Class
 * @file keypoint_detection.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__KEYPOINT_DETECTION_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__KEYPOINT_DETECTION_HPP_
#include <string>
#include <vector>
#include "openvino_wrapper_lib/inferences/model_inference.hpp"

namespace Models
{
class KeypointDetectionYolo26Model;
}

namespace openvino_wrapper_lib
{
/**
 * @class KeypointDetectionResult
 * @brief Class for storing a detected object with its keypoints, for example a person with body joints.
 */
class KeypointDetectionResult : public Result
{
public:
  explicit KeypointDetectionResult(const cv::Rect& location) : Result(location)
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
   * @brief Get the keypoints in frame coordinates, in the order defined by the model.
   */
  inline const std::vector<cv::Point2f>& getKeypoints() const
  {
    return keypoints_;
  }
  /**
   * @brief Get the confidence of each keypoint.
   */
  inline const std::vector<float>& getKeypointConfidences() const
  {
    return keypoint_confidences_;
  }
  inline void addKeypoint(const cv::Point2f& keypoint, float confidence)
  {
    keypoints_.push_back(keypoint);
    keypoint_confidences_.push_back(confidence);
  }

private:
  std::string label_ = "";
  float confidence_ = -1;
  std::vector<cv::Point2f> keypoints_;
  std::vector<float> keypoint_confidences_;
};

/**
 * @brief Inference that detects objects and their keypoints (pose estimation).
 */
using KeypointDetection = ModelInference<Models::KeypointDetectionYolo26Model, KeypointDetectionResult>;
}  // namespace openvino_wrapper_lib
#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__KEYPOINT_DETECTION_HPP_
