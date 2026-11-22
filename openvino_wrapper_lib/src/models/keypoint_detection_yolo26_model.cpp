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

#include <memory>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/models/keypoint_detection_yolo26_model.hpp"

const std::string Models::KeypointDetectionYolo26Model::getModelCategory() const
{
  return "Keypoint Detection Yolo26";
}

void Models::KeypointDetectionYolo26Model::checkOutputShape(const ov::Shape& shape)
{
  if (shape.size() != 3 || shape[2] <= 6 || (shape[2] - 6) % 3 != 0) {
    throw std::logic_error(
        "YOLO26 keypoint detection needs the end-to-end output [1, N, 6 + 3 * keypoints], but "
        "the output of " +
        getModelName() + " is " + shape.to_string() + ". Export the model with nms=False.");
  }
}

bool Models::KeypointDetectionYolo26Model::fetchResults(
    const std::shared_ptr<Engines::Engine>& engine, std::vector<openvino_wrapper_lib::KeypointDetectionResult>& results,
    float confidence_thresh)
{
  ov::Tensor output_tensor = engine->getRequest().get_output_tensor();
  const ov::Shape output_shape = output_tensor.get_shape();
  const float* detections = output_tensor.data<float>();
  const float rx = getFrameResizeRatioWidth();
  const float ry = getFrameResizeRatioHeight();

  for (size_t i = 0; i < output_shape[1]; i++) {
    const float* detection = detections + i * output_shape[2];
    const float confidence = detection[4];
    if (confidence <= confidence_thresh) {
      continue;
    }
    cv::Rect rec(cv::Point(rx * detection[0], ry * detection[1]), cv::Point(rx * detection[2], ry * detection[3]));
    openvino_wrapper_lib::KeypointDetectionResult result(rec & cv::Rect(cv::Point(0, 0), getFrameSize()));
    result.setConfidence(confidence);
    result.setLabel(getLabel(static_cast<size_t>(detection[5])));
    for (size_t k = 6; k < output_shape[2]; k += 3) {
      result.addKeypoint(cv::Point2f(rx * detection[k], ry * detection[k + 1]), detection[k + 2]);
    }
    results.push_back(result);
  }

  return true;
}
