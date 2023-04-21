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
 * @brief A header file with declaration for VehicleAttribsDetectionModel Class
 * @file vehicle_attribs_detection_model.h
 */
#ifndef OPENVINO_WRAPPER_LIB__MODELS__VEHICLE_ATTRIBS_DETECTION_MODEL_HPP_
#define OPENVINO_WRAPPER_LIB__MODELS__VEHICLE_ATTRIBS_DETECTION_MODEL_HPP_
#include <string>
#include "openvino_wrapper_lib/models/base_model.hpp"
namespace Models
{
/**
 * @class VehicleAttribsDetectionModel
 * @brief This class generates the vehicle attributes detection model.
 */
class VehicleAttribsDetectionModel : public BaseModel
{
public:
  VehicleAttribsDetectionModel(const std::string& label_loc, const std::string & model_loc, int batch_size = 1);
  inline const std::string getInputName() {return input_tensor_name_;}
  inline const std::string getColorOutputName() {return color_output_;}
  inline const std::string getTypeOutputName() {return type_output_;}
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;

protected:
  bool updateLayerProperty(std::shared_ptr<ov::Model>&) override;
  std::string color_output_;
  std::string type_output_;
};
}  // namespace Models
#endif  // OPENVINO_WRAPPER_LIB__MODELS__VEHICLE_ATTRIBS_DETECTION_MODEL_HPP_
