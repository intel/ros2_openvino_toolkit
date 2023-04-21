// Copyright (c) 2018-2022 Intel Corporation
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
 * @brief A header file with declaration for HeadPoseDetectionModel Class
 * @file head_pose_detection_model.h
 */

#ifndef OPENVINO_WRAPPER_LIB__MODELS__HEAD_POSE_DETECTION_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__HEAD_POSE_DETECTION_MODEL_HPP_

#include <string>
#include "openvino_wrapper_lib/models/base_model.hpp"

namespace Models
{
/**
 * @class HeadPoseDetectionModel
 * @brief This class generates the headpose detection model.
 */
class HeadPoseDetectionModel : public BaseModel
{
public:
  HeadPoseDetectionModel(const std::string& label_loc, const std::string & model_loc, int batch_size = 1);

  /**
   * @brief Get the output angle roll.
   * @return Roll value.
   */
  inline const std::string getOutputOutputAngleR() const
  {
    return output_angle_r_;
  }
  /**
   * @brief Get the output angle pitch.
   * @return Pitch value.
   */
  inline const std::string getOutputOutputAngleP() const
  {
    return output_angle_p_;
  }
  /**
   * @brief Get the output angle yawl.
   * @return Yawl value.
   */
  inline const std::string getOutputOutputAngleY() const
  {
    return output_angle_y_;
  }
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;


private:
  std::string output_angle_r_ = "angle_r_fc";
  std::string output_angle_p_ = "angle_p_fc";
  std::string output_angle_y_ = "angle_y_fc";
};
}  // namespace Models

#endif  // OPENVINO_WRAPPER_LIB__MODELS__HEAD_POSE_DETECTION_MODEL_HPP_
