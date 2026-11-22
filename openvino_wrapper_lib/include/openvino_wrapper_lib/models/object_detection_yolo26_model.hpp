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

#ifndef OPENVINO_WRAPPER_LIB__MODELS__OBJECT_DETECTION_YOLO26_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__OBJECT_DETECTION_YOLO26_MODEL_HPP_
#include <string>
#include <memory>
#include <vector>
#include "openvino_wrapper_lib/models/object_detection_yolov5_model.hpp"

namespace Models
{
/**
 * @class ObjectDetectionYolo26Model
 * @brief Ultralytics YOLO26 detection model with the end-to-end (NMS-free) head,
 * whose output is [1, N, 6]: x1, y1, x2, y2, confidence, class id.
 */
class ObjectDetectionYolo26Model : public ObjectDetectionYolov5Model
{
  using Result = openvino_wrapper_lib::ObjectDetectionResult;

public:
  ObjectDetectionYolo26Model(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);

  bool fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                    std::vector<openvino_wrapper_lib::ObjectDetectionResult>& results,
                    const float& confidence_thresh = 0.3, const bool& enable_roi_constraint = false) override;

  const std::string getModelCategory() const override;
  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;
};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__OBJECT_DETECTION_YOLO26_MODEL_HPP_
