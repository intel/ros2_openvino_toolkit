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
 * @brief A header file with declaration for PersonReidentificationModel Class
 * @file face_detection_model.h
 */
#ifndef DYNAMIC_VINO_LIB__MODELS__PERSON_REIDENTIFICATION_MODEL_HPP_
#define DYNAMIC_VINO_LIB__MODELS__PERSON_REIDENTIFICATION_MODEL_HPP_
#include <string>
#include "dynamic_vino_lib/models/base_model.hpp"
namespace Models {
/**
 * @class PersonReidentificationModel
 * @brief This class generates the person reidentification model.
 */
class PersonReidentificationModel : public BaseModel {
 public:
  PersonReidentificationModel(const std::string&, int, int, int);
  inline const std::string getInputName() { return input_; }
  inline const std::string getOutputName() { return output_; }
  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelName() const override;
 protected:
  void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr&) override;
  void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;
  std::string input_;
  std::string output_;
};
}  // namespace Models
#endif  // DYNAMIC_VINO_LIB__MODELS__PERSON_REIDENTIFICATION_MODEL_HPP_