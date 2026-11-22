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
#include "openvino_wrapper_lib/models/depth_estimation_yolo26_model.hpp"

const std::string Models::DepthEstimationYolo26Model::getModelCategory() const
{
  return "Depth Estimation Yolo26";
}

void Models::DepthEstimationYolo26Model::checkOutputShape(const ov::Shape& shape)
{
  if (shape.size() != 4 || shape[1] != 1) {
    throw std::logic_error("YOLO26 depth estimation needs the output [1, 1, H, W], but the output of " +
                           getModelName() + " is " + shape.to_string() + ".");
  }
}

bool Models::DepthEstimationYolo26Model::fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                                                      std::vector<openvino_wrapper_lib::DepthEstimationResult>& results,
                                                      float)
{
  ov::Tensor output_tensor = engine->getRequest().get_output_tensor();
  const ov::Shape output_shape = output_tensor.get_shape();
  const cv::Mat output(static_cast<int>(output_shape[2]), static_cast<int>(output_shape[3]), CV_32FC1,
                       output_tensor.data<float>());

  cv::Mat depth;
  cv::resize(output(getFrameAreaInOutput(output.size())), depth, getFrameSize());
  openvino_wrapper_lib::DepthEstimationResult result(cv::Rect(cv::Point(0, 0), getFrameSize()));
  result.setDepth(depth);
  results.push_back(result);

  return true;
}
