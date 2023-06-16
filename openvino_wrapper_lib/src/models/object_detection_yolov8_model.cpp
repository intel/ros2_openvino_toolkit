// Copyright (c) 2023 Intel Corporation
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

#include "openvino_wrapper_lib/models/object_detection_yolov8_model.hpp"

Models::ObjectDetectionYolov8Model::ObjectDetectionYolov8Model(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: ObjectDetectionYolov5Model(label_loc, model_loc, max_batch_size)
{
  //setKeepInputShapeRatio(true);
  setHasConfidenceOutput(false);
  setExpectedFrameSize({640, 640});
}

