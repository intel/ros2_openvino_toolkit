/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief A header file with declaration for FaceDetectionModel Class
 * @file face_detection_model.h
 */

#ifndef DYNAMIC_VINO_LIB__MODELS__FACE_DETECTION_MODEL_HPP_
#define DYNAMIC_VINO_LIB__MODELS__FACE_DETECTION_MODEL_HPP_

#include <string>
#include "dynamic_vino_lib/models/object_detection_model.hpp"

namespace Models {
/**
 * @class FaceDetectionModel
 * @brief This class generates the face detection model.
 */
class FaceDetectionModel : public ObjectDetectionModel {
 public:
  FaceDetectionModel(const std::string&, int, int, int);
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelName() const override;
};
}  // namespace Models

#endif  // DYNAMIC_VINO_LIB__MODELS__FACE_DETECTION_MODEL_HPP_
