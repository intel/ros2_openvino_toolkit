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
 * @brief A header file with declaration for AgeGenderDetectionModel Class
 * @file age_gender_detection_model.h
 */

#ifndef DYNAMIC_VINO_LIB__MODELS__AGE_GENDER_DETECTION_MODEL_HPP_
#define DYNAMIC_VINO_LIB__MODELS__AGE_GENDER_DETECTION_MODEL_HPP_

#include <string>
#include "dynamic_vino_lib/models/base_model.hpp"

namespace Models {
/**
 * @class AgeGenderDetectionModel
 * @brief This class generates the age gender detection model.
 */
class AgeGenderDetectionModel : public BaseModel {
 public:
  AgeGenderDetectionModel(const std::string&, int, int, int);
  /**
   * @brief Get the input name.
   * @return Input name.
   */
  inline const std::string getInputName() const { return input_; }
  /**
   * @brief Get the age from the detection reuslt.
   * @return Detected age.
   */
  inline const std::string getOutputAgeName() const { return output_age_; }
  /**
   * @brief Get the gender from the detection reuslt.
   * @return Detected gender.
   */
  inline const std::string getOutputGenderName() const {
    return output_gender_;
  }
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelName() const override;

 protected:
  void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr&) override;
  void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

 private:
  std::string input_;
  std::string output_age_;
  std::string output_gender_;
};
}  // namespace Models

#endif  // DYNAMIC_VINO_LIB__MODELS__AGE_GENDER_DETECTION_MODEL_HPP_
