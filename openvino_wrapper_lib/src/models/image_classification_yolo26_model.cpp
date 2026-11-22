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

#include <algorithm>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/models/image_classification_yolo26_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"

Models::ImageClassificationYolo26Model::ImageClassificationYolo26Model(
    const Params::ParamManager::InferenceRawData& config)
  : Yolo26Model(config)
{
  setKeepInputShapeRatio(false);
}

const std::string Models::ImageClassificationYolo26Model::getModelCategory() const
{
  return "Image Classification Yolo26";
}

void Models::ImageClassificationYolo26Model::checkOutputShape(const ov::Shape& shape)
{
  if (shape.size() != 2) {
    throw std::logic_error("YOLO26 classification needs the output [1, classes], but the output of " + getModelName() +
                           " is " + shape.to_string() + ".");
  }
  if (!getLabels().empty() && getLabels().size() != shape[1]) {
    slog::warn << "The label file has " << getLabels().size() << " labels, but " << getModelName() << " has "
               << shape[1] << " classes." << slog::endl;
  }
}

bool Models::ImageClassificationYolo26Model::enqueue(const std::shared_ptr<Engines::Engine>& engine,
                                                     const cv::Mat& frame, const cv::Rect& input_frame_loc)
{
  setFrameSize(frame.cols, frame.rows);
  const int side = std::min(frame.cols, frame.rows);
  const cv::Rect center((frame.cols - side) / 2, (frame.rows - side) / 2, side, side);
  return matToBlob(frame(center), input_frame_loc, 1, 0, engine);
}

bool Models::ImageClassificationYolo26Model::fetchResults(
    const std::shared_ptr<Engines::Engine>& engine,
    std::vector<openvino_wrapper_lib::ImageClassificationResult>& results, float confidence_thresh)
{
  ov::Tensor output_tensor = engine->getRequest().get_output_tensor();
  const float* probabilities = output_tensor.data<float>();
  const size_t class_count = output_tensor.get_shape()[1];

  std::vector<size_t> class_ids(class_count);
  std::iota(class_ids.begin(), class_ids.end(), 0);
  const size_t top = std::min<size_t>(5, class_count);
  std::partial_sort(class_ids.begin(), class_ids.begin() + top, class_ids.end(),
                    [&](size_t a, size_t b) { return probabilities[a] > probabilities[b]; });

  for (size_t i = 0; i < top && probabilities[class_ids[i]] > confidence_thresh; i++) {
    openvino_wrapper_lib::ImageClassificationResult result(cv::Rect(cv::Point(0, 0), getFrameSize()));
    result.setConfidence(probabilities[class_ids[i]]);
    result.setLabel(getLabel(class_ids[i]));
    results.push_back(result);
  }

  return true;
}
