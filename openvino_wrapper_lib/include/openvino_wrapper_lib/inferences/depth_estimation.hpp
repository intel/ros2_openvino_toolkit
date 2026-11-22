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

/**
 * @brief A header file with declaration for DepthEstimation Class
 * @file depth_estimation.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__DEPTH_ESTIMATION_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__DEPTH_ESTIMATION_HPP_
#include "openvino_wrapper_lib/inferences/model_inference.hpp"

namespace Models
{
class DepthEstimationYolo26Model;
}

namespace openvino_wrapper_lib
{
/**
 * @class DepthEstimationResult
 * @brief Class for storing the depth map of a frame. The location of the result is the whole frame.
 */
class DepthEstimationResult : public Result
{
public:
  explicit DepthEstimationResult(const cv::Rect& location) : Result(location)
  {
  }
  /**
   * @brief Get the depth map (CV_32FC1, in meters) with the size of the frame.
   */
  inline cv::Mat getDepth() const
  {
    return depth_;
  }
  inline void setDepth(const cv::Mat& depth)
  {
    depth_ = depth;
  }

private:
  cv::Mat depth_;
};

/**
 * @brief Inference that estimates the depth of each pixel of the frame.
 */
using DepthEstimation = ModelInference<Models::DepthEstimationYolo26Model, DepthEstimationResult>;
}  // namespace openvino_wrapper_lib
#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__DEPTH_ESTIMATION_HPP_
