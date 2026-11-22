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

#ifndef OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_INSTANCE_YOLO26_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_INSTANCE_YOLO26_MODEL_HPP_
#include <string>
#include <memory>
#include <vector>
#include "openvino_wrapper_lib/models/object_segmentation_instance_model.hpp"

namespace Models
{
/**
 * @class ObjectSegmentationInstanceYolo26Model
 * @brief Ultralytics YOLO26 instance segmentation model with the end-to-end (NMS-free) head, whose
 * detection output is [1, N, 6 + M]: x1, y1, x2, y2, confidence, class id and M mask coefficients.
 */
class ObjectSegmentationInstanceYolo26Model : public ObjectSegmentationInstanceModel
{
public:
  explicit ObjectSegmentationInstanceYolo26Model(const Params::ParamManager::InferenceRawData& config);

  const std::string getModelCategory() const override;
  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;

protected:
  std::vector<int> decodeDetections(ov::InferRequest& request, float confidence_thresh, std::vector<cv::Rect>& boxes,
                                    std::vector<int>& class_ids, std::vector<float>& confidences,
                                    std::vector<cv::Mat>& mask_confs) override;
};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_INSTANCE_YOLO26_MODEL_HPP_
