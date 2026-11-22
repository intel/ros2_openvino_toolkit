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

#ifndef OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_YOLO26_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_YOLO26_MODEL_HPP_
#include <string>
#include <memory>
#include "openvino_wrapper_lib/models/object_segmentation_model.hpp"

namespace Models
{
/**
 * @class ObjectSegmentationYolo26Model
 * @brief Ultralytics YOLO26 semantic segmentation model, whose output is a class map [1, H, W].
 */
class ObjectSegmentationYolo26Model : public ObjectSegmentationModel
{
public:
  ObjectSegmentationYolo26Model(const std::string& label_loc, const std::string& model_loc, int batch_size = 1);

  bool enqueue(const std::shared_ptr<Engines::Engine>&, const cv::Mat&, const cv::Rect&) override;

  const std::string getModelCategory() const override;
  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;
};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__OBJECT_SEGMENTATION_YOLO26_MODEL_HPP_
