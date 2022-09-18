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
 * @brief A header file with declaration for AgeGenderDetection Class
 * @file age_gender_recignition.h
 */
#ifndef DYNAMIC_VINO_LIB__INFERENCES__AGE_GENDER_DETECTION_HPP_
#define DYNAMIC_VINO_LIB__INFERENCES__AGE_GENDER_DETECTION_HPP_

#include <people_msgs/msg/age_gender.hpp>
#include <people_msgs/msg/age_gender_stamped.hpp>
#include <memory>
#include <string>
#include <vector>

#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/inferences/base_inference.hpp"
#include "dynamic_vino_lib/models/age_gender_detection_model.hpp"
// #include "inference_engine.hpp"
#include "openvino/openvino.hpp"
#include "opencv2/opencv.hpp"

namespace Outputs
{
class BaseOuput;
}
namespace dynamic_vino_lib
{
/**
 * @class AgeGenderResult
 * @brief Class for storing and processing age and gender detection result.
 */
class AgeGenderResult : public Result
{
public:
  explicit AgeGenderResult(const cv::Rect & location);
  /**
   * @brief Get the age of the detected person from the result.
   * @return The predictea age.
   */
  float getAge() const
  {
    return age_;
  }
  /**
   * @brief Get the possibility that the detected person is a
   * male from the result.
   * @return The possibility that the detected person is a male.
   */
  float getMaleProbability() const
  {
    return male_prob_;
  }

  float age_ = -1;
  float male_prob_ = -1;
};

/**
 * @class AgeGenderDetection
 * @brief Class to load age and gender detection model and perform
   age and gender detection.
 */
class AgeGenderDetection : public BaseInference
{
public:
  using Result = dynamic_vino_lib::AgeGenderResult;
  AgeGenderDetection();
  ~AgeGenderDetection() override;
  /**
   * @brief Load the age gender detection model.
   */
  void loadNetwork(std::shared_ptr<Models::AgeGenderDetectionModel>);
  /**
   * @brief Enqueue a frame to this class.
   * The frame will be buffered but not infered yet.
   * @param[in] frame The frame to be enqueued.
   * @param[in] input_frame_loc The location of the enqueued frame with respect
   * to the frame generated by the input device.
   * @return Whether this operation is successful.
   */
  bool enqueue(const cv::Mat & frame, const cv::Rect &) override;
  /**
   * @brief Start inference for all buffered frames.
   * @return Whether this operation is successful.
   */
  bool submitRequest() override;
  /**
   * @brief This function will fetch the results of the previous inference and
   * stores the results in a result buffer array. All buffered frames will be
   * cleared.
   * @return Whether the Inference object fetches a result this time
   */
  bool fetchResults() override;
  /**
   * @brief Get the length of the buffer result array.
   * @return The length of the buffer result array.
   */
  int getResultsLength() const override;
  /**
   * @brief Get the location of result with respect
   * to the frame generated by the input device.
   * @param[in] idx The index of the result.
   */
  const dynamic_vino_lib::Result * getLocationResult(int idx) const override;
  /**
   * @brief Get the name of the Inference instance.
   * @return The name of the Inference instance.
   */
  const std::string getName() const override;
  /**
   * @brief Show the observed detection result either through image window
   * or ROS topic.
   */
  void observeOutput(const std::shared_ptr<Outputs::BaseOutput> & output) override;

  const std::vector<cv::Rect> getFilteredROIs(
    const std::string filter_conditions) const override;

private:
  std::shared_ptr<Models::AgeGenderDetectionModel> valid_model_;
  std::vector<Result> results_;
};
}  // namespace dynamic_vino_lib

#endif  // DYNAMIC_VINO_LIB__INFERENCES__AGE_GENDER_DETECTION_HPP_
