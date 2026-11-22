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

#ifndef OPENVINO_WRAPPER_LIB__MODELS__YOLO26_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__YOLO26_MODEL_HPP_
#include <string>
#include <memory>
#include "openvino_wrapper_lib/models/base_model.hpp"

namespace Models
{
/**
 * @brief Insert the image pre-processing of Ultralytics models into a model: the input becomes a
 * BGR u8 NHWC frame, which is converted to RGB, float (0-1) and NCHW. All outputs become float.
 * @param[in] model The model to be changed.
 * @param[in] input_name The tensor name of the image input.
 */
void addYoloPreprocessing(std::shared_ptr<ov::Model>& model, const std::string& input_name);

/**
 * @class Yolo26Model
 * @brief Base class of the Ultralytics YOLO26 models with one image input and one output.
 * The frame is padded at the right or bottom to the ratio of the input and then resized.
 */
class Yolo26Model : public BaseModel
{
public:
  explicit Yolo26Model(const Params::ParamManager::InferenceRawData& config);

  bool enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
               const cv::Rect& input_frame_loc) override;

protected:
  bool updateLayerProperty(std::shared_ptr<ov::Model>& model) override;

  /**
   * @brief Check that the output of the model fits the task.
   * @param[in] shape The shape of the output.
   * @throw std::logic_error when the shape is not supported.
   */
  virtual void checkOutputShape(const ov::Shape& shape) = 0;
};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__YOLO26_MODEL_HPP_
