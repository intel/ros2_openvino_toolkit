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

#ifndef OPENVINO_WRAPPER_LIB__MODELS__IMAGE_CLASSIFICATION_YOLO26_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__IMAGE_CLASSIFICATION_YOLO26_MODEL_HPP_
#include <memory>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/inferences/image_classification.hpp"
#include "openvino_wrapper_lib/models/yolo26_model.hpp"

namespace Models
{
/**
 * @class ImageClassificationYolo26Model
 * @brief Ultralytics YOLO26 classification model, whose output is [1, C]: the probability of each class.
 * The model sees the center square of the frame.
 */
class ImageClassificationYolo26Model : public Yolo26Model
{
public:
  explicit ImageClassificationYolo26Model(const Params::ParamManager::InferenceRawData& config);

  bool enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
               const cv::Rect& input_frame_loc) override;

  /**
   * @brief Fetch the classes with a probability above the threshold, at most five, most probable first.
   */
  bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                    std::vector<openvino_wrapper_lib::ImageClassificationResult>& results, float confidence_thresh);

  const std::string getModelCategory() const override;

protected:
  void checkOutputShape(const ov::Shape& shape) override;
};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__IMAGE_CLASSIFICATION_YOLO26_MODEL_HPP_
