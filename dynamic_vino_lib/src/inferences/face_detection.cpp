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
 * @brief a header file with declaration of FaceDetection class and
 * FaceDetectionResult class
 * @file face_detection.cpp
 */

#include <memory>
#include <string>
#include <vector>

#include "dynamic_vino_lib/inferences/face_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// FaceDetectionResult
dynamic_vino_lib::FaceDetectionResult::FaceDetectionResult(
    const cv::Rect& location)
    : ObjectDetectionResult(location){}

// FaceDetection
dynamic_vino_lib::FaceDetection::FaceDetection(double show_output_thresh)
    : ObjectDetection(show_output_thresh){}

