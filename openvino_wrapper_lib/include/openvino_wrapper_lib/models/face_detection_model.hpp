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
 * @brief A header file with declaration for FaceDetectionModel Class
 * @file face_detection_model.h
 */

#ifndef OPENVINO_WRAPPER_LIB__MODELS__FACE_DETECTION_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__FACE_DETECTION_MODEL_HPP_

#include <string>
#include "openvino_wrapper_lib/models/base_model.hpp"

namespace Models
{
/**
 * @class FaceDetectionModel
 * @brief This class generates the face detection model.
 */
class FaceDetectionModel : public ObjectDetectionModel
{
public:
  FaceDetectionModel(const std::string& label_loc, const std::string & model_loc, int batch_size = 1);
  //void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr &) override;
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
};
}  // namespace Models

#endif  // OPENVINO_WRAPPER_LIB__MODELS__FACE_DETECTION_MODEL_HPP_
