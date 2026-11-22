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
 * @brief A header file with declaration for ImageClassification Class
 * @file image_classification.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__IMAGE_CLASSIFICATION_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__IMAGE_CLASSIFICATION_HPP_
#include <string>
#include "openvino_wrapper_lib/inferences/model_inference.hpp"

namespace Models
{
class ImageClassificationYolo26Model;
}

namespace openvino_wrapper_lib
{
/**
 * @class ImageClassificationResult
 * @brief Class for storing one class of a classified frame. The results of a frame are sorted
 * by descending confidence and their location is the whole frame.
 */
class ImageClassificationResult : public Result
{
public:
  explicit ImageClassificationResult(const cv::Rect& location) : Result(location)
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

private:
  std::string label_ = "";
  float confidence_ = -1;
};

/**
 * @brief Inference that classifies the whole frame.
 */
using ImageClassification = ModelInference<Models::ImageClassificationYolo26Model, ImageClassificationResult>;
}  // namespace openvino_wrapper_lib
#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__IMAGE_CLASSIFICATION_HPP_
