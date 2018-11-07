
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
 * @brief a header file with declaration of FaceDetectionModel class
 * @file face_detection_model.cpp
 */

#include <string>

#include "dynamic_vino_lib/models/face_detection_model.hpp"
#include "dynamic_vino_lib/slog.hpp"

// Validated Face Detection Network
Models::FaceDetectionModel::FaceDetectionModel(const std::string& model_loc,
                                               int input_num, int output_num,
                                               int max_batch_size)
    : ObjectDetectionModel(model_loc, input_num, output_num, max_batch_size){}

const std::string Models::FaceDetectionModel::getModelName() const {
  return "Face Detection";
}
