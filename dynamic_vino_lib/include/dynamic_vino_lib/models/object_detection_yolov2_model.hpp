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
 * @brief A header file with declaration for ObjectDetectionModel Class
 * @file face_detection_model.h
 */
#ifndef DYNAMIC_VINO_LIB__MODELS__OBJECT_DETECTION_YOLOV2_MODEL_HPP_
#define DYNAMIC_VINO_LIB__MODELS__OBJECT_DETECTION_YOLOV2_MODEL_HPP_
#include <string>
#include <memory>
#include <vector>
#include "dynamic_vino_lib/models/base_model.hpp"
namespace Models
{
/**
 * @class ObjectDetectionModel
 * @brief This class generates the face detection model.
 */
class ObjectDetectionYolov2Model : public ObjectDetectionModel
{
  using Result = dynamic_vino_lib::ObjectDetectionResult;

public:
  ObjectDetectionYolov2Model(const std::string &, int, int, int);

  bool fetchResults(
    const std::shared_ptr<Engines::Engine> & engine,
    std::vector<dynamic_vino_lib::ObjectDetectionResult> & results,
    const float & confidence_thresh = 0.3,
    const bool & enable_roi_constraint = false) override;

  bool enqueue(
    const std::shared_ptr<Engines::Engine> & engine,
    const cv::Mat & frame,
    const cv::Rect & input_frame_loc) override;

  bool matToBlob(
    const cv::Mat & orig_image, const cv::Rect &, float scale_factor,
    int batch_index, const std::shared_ptr<Engines::Engine> & engine) override;
  inline const std::string getInputName()
  {
    return input_;
  }
  inline const std::string getOutputName()
  {
    return output_;
  }

  InferenceEngine::CNNLayerPtr getLayer() {return output_layer_;}
  InferenceEngine::InputInfo::Ptr getInputInfo() {return input_info_;}

  /**
   * @brief Get the name of this detection model.
   * @return Name of the model.
   */
  const std::string getModelName() const override;

protected:
  void checkLayerProperty(const InferenceEngine::CNNNetReader::Ptr &) override;
  void setLayerProperty(InferenceEngine::CNNNetReader::Ptr) override;
  int getEntryIndex(int side, int lcoords, int lclasses, int location, int entry);

  std::string input_;
  std::string output_;
  InferenceEngine::CNNLayerPtr output_layer_;
  InferenceEngine::InputInfo::Ptr input_info_;
};
}  // namespace Models
#endif  // DYNAMIC_VINO_LIB__MODELS__OBJECT_DETECTION_YOLOV2_MODEL_HPP_
