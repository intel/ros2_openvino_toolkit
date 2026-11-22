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
#include "openvino_wrapper_lib/models/object_segmentation_instance_yolo26_model.hpp"

Models::ObjectSegmentationInstanceYolo26Model::ObjectSegmentationInstanceYolo26Model(
    const Params::ParamManager::InferenceRawData& config)
  : ObjectSegmentationInstanceModel(config)
{
}

const std::string Models::ObjectSegmentationInstanceYolo26Model::getModelCategory() const
{
  return "Object Segmentation - Yolo26";
}

bool Models::ObjectSegmentationInstanceYolo26Model::updateLayerProperty(std::shared_ptr<ov::Model>& model)
{
  if (!ObjectSegmentationInstanceModel::updateLayerProperty(model)) {
    return false;
  }
  const ov::Shape det_shape = model->output(getOutputName("detection")).get_shape();
  const ov::Shape mask_shape = model->output(getOutputName("masks")).get_shape();
  if (det_shape[2] != 6 + mask_shape[1]) {
    throw std::logic_error("YOLO26 instance segmentation needs the end-to-end output [1, N, " +
                           std::to_string(6 + mask_shape[1]) + "], but the output of " + getModelName() + " is " +
                           det_shape.to_string() + ". Export the model with nms=False.");
  }
  return true;
}

std::vector<int> Models::ObjectSegmentationInstanceYolo26Model::decodeDetections(
    ov::InferRequest& request, float confidence_thresh, std::vector<cv::Rect>& boxes, std::vector<int>& class_ids,
    std::vector<float>& confidences, std::vector<cv::Mat>& mask_confs)
{
  ov::Tensor output_tensor = request.get_tensor(getOutputName("detection"));
  const ov::Shape output_shape = output_tensor.get_shape();
  float* detections = output_tensor.data<float>();
  const int mask_channels = static_cast<int>(output_shape[2]) - 6;

  std::vector<int> kept;
  for (size_t i = 0; i < output_shape[1]; i++) {
    float* detection = detections + i * output_shape[2];
    if (detection[4] <= confidence_thresh) {
      continue;
    }
    kept.push_back(static_cast<int>(boxes.size()));
    boxes.emplace_back(cv::Point(detection[0], detection[1]), cv::Point(detection[2], detection[3]));
    confidences.push_back(detection[4]);
    class_ids.push_back(static_cast<int>(detection[5]));
    mask_confs.emplace_back(1, mask_channels, CV_32F, detection + 6);
  }
  return kept;
}
