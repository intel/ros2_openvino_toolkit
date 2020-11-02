// Copyright (c) 2018 Intel Corporation
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
 * @brief A header file with declaration for ObjectSegmentationModel Class
 * @file face_detection_model.h
 */
#ifndef DYNAMIC_VINO_LIB__MODELS__OBJECT_SEGMENTATION_MODEL_HPP_
#define DYNAMIC_VINO_LIB__MODELS__OBJECT_SEGMENTATION_MODEL_HPP_
#include <string>
#include "dynamic_vino_lib/models/base_model.hpp"
namespace Models
{
/**
 * @class ObjectSegmentationModel
 * @brief This class generates the object segmentation model.
 */
class ObjectSegmentationModel : public BaseModel
{
public:
  ObjectSegmentationModel(const std::string & model_loc, int batch_size = 1);
  inline int getMaxProposalCount() const
  {
    return max_proposal_count_;
  }
  inline int getObjectSize() const
  {
    return object_size_;
  }
  /*inline const std::string getInputName()
  {
    return input_;
  }
  inline const std::string getDetectionOutputName()
  {
    return detection_output_;
  }
  inline const std::string getMaskOutputName()
  {
    return mask_output_;
  }
  inline size_t getOutputChannelSize() const
  {
    return output_channels_;
  }
  inline size_t getOutputHeight() const
  {
    return output_height_;
  }
  inline size_t getOutputWidth() const
  {
    return output_width_;
  }*/

  bool enqueue(const std::shared_ptr<Engines::Engine> & ,const cv::Mat &,
    const cv::Rect & ) override;

  bool matToBlob(
    const cv::Mat & , const cv::Rect &, float ,
    int , const std::shared_ptr<Engines::Engine> & );

  /**
   * @brief Get the name of this segmentation model.
   * @return Name of the model.
   */
  const std::string getModelCategory() const override;
  bool updateLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

protected:
  //void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr &) override;
  //void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;

private:
  int max_proposal_count_;
  int object_size_;
  //std::string input_ = "data";
  //std::string mask_output_ = "masks";
  //std::string detection_output_ = "detection";

  //todo: output size
  size_t output_channels_ = 0;
  size_t output_height_ = 0;
  size_t output_width_ = 0;
  InferenceEngine::InputsDataMap input_info_;
  //InferenceEngine::InputInfo::Ptr input_info_ = nullptr;
  InferenceEngine::OutputsDataMap output_info_;

};
}  // namespace Models
#endif  // DYNAMIC_VINO_LIB__MODELS__OBJECT_SEGMENTATION_MODEL_HPP_
