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
 * @brief A header file with declaration for ObjectDetectionModel Class
 * @file face_detection_model.h
 */
#ifndef OPENVINO_WRAPPER_LIB__MODELS__OBJECT_DETECTION_SSD_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__OBJECT_DETECTION_SSD_MODEL_HPP_
#include <string>
#include <memory>
#include <vector>
#include "openvino_wrapper_lib/models/base_model.hpp"
namespace Models
{
/**
 * @class ObjectDetectionModel
 * @brief This class generates the face detection model.
 */
class ObjectDetectionSSDModel : public ObjectDetectionModel
{
  using Result = openvino_wrapper_lib::ObjectDetectionResult;

public:
  ObjectDetectionSSDModel() {};

  bool fetchResults(
    const std::shared_ptr<Engines::Engine> & engine,
    std::vector<openvino_wrapper_lib::ObjectDetectionResult> & results,
    const float & confidence_thresh = 0.3,
    const bool & enable_roi_constraint = false) override;

  bool enqueue(
    const std::shared_ptr<Engines::Engine> & engine,
    const cv::Mat & frame,
    const cv::Rect & input_frame_loc) override;

  bool matToBlob(
    const cv::Mat & orig_image, const cv::Rect &, float scale_factor,
    int batch_index, const std::shared_ptr<Engines::Engine> & engine) override;

  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;

  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;

};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__OBJECT_DETECTION_SSD_MODEL_HPP_
