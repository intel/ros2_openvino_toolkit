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

#include <string>
#include <memory>
#include <vector>
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/inferences/object_detection.hpp"
#include "openvino_wrapper_lib/models/object_detection_yolo26_model.hpp"

Models::ObjectDetectionYolo26Model::ObjectDetectionYolo26Model(const std::string& label_loc,
                                                               const std::string& model_loc, int max_batch_size)
  : ObjectDetectionYolov5Model(label_loc, model_loc, max_batch_size)
{
}

bool Models::ObjectDetectionYolo26Model::updateLayerProperty(std::shared_ptr<ov::Model>& model)
{
  if (!ObjectDetectionYolov5Model::updateLayerProperty(model)) {
    return false;
  }
  if (getObjectSize() != 6) {
    throw std::logic_error("YOLO26 detection needs the end-to-end output [1, N, 6], but the output of " +
                           getModelName() + " is " + model->output().get_shape().to_string() +
                           ". Export the model with nms=False.");
  }
  return true;
}

const std::string Models::ObjectDetectionYolo26Model::getModelCategory() const
{
  return "Object Detection Yolo26";
}

bool Models::ObjectDetectionYolo26Model::fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                                                      std::vector<openvino_wrapper_lib::ObjectDetectionResult>& results,
                                                      const float& confidence_thresh, const bool& enable_roi_constraint)
{
  ov::Tensor output_tensor = engine->getRequest().get_output_tensor();
  const ov::Shape output_shape = output_tensor.get_shape();
  const float* detections = output_tensor.data<float>();
  const double rx = getFrameResizeRatioWidth();
  const double ry = getFrameResizeRatioHeight();

  for (size_t i = 0; i < output_shape[1]; i++) {
    const float* detection = detections + i * output_shape[2];
    const float confidence = detection[4];
    if (confidence <= confidence_thresh) {
      continue;
    }
    cv::Rect rec(cv::Point(rx * detection[0], ry * detection[1]), cv::Point(rx * detection[2], ry * detection[3]));
    if (enable_roi_constraint) {
      rec &= cv::Rect(cv::Point(0, 0), getFrameSize());
    }
    Result result(rec);
    result.setConfidence(confidence);
    result.setLabel(getLabel(static_cast<size_t>(detection[5])));
    results.push_back(result);
  }

  return true;
}
