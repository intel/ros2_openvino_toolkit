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
* @brief A header file with declaration for RvizOutput Class
* @file image_window_output.h
*/

#ifndef DYNAMIC_VINO_LIB__OUTPUTS__RVIZ_OUTPUT_HPP_
#define DYNAMIC_VINO_LIB__OUTPUTS__RVIZ_OUTPUT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <string>
#include <memory>

#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/outputs/image_window_output.hpp"

namespace Outputs
{
/**
 * @class RvizOutput
 * @brief This class handles and shows the detection result with rviz.
 */
class RvizOutput : public BaseOutput
{
public:
  explicit RvizOutput(std::string output_name);
  /**
   * @brief Construct frame for rviz
   * @param[in] A frame.
   */
  void feedFrame(const cv::Mat &) override;
  /**
   * @brief Show all the contents generated by the accept
   * functions with rviz.
   */
  void handleOutput() override;
  /**
   * @brief Generate rviz output content according to
   * the face reidentification result.
   * @param[in] A face reidentification result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::FaceReidentificationResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the landmarks detection result.
   * @param[in] A landmarks detection result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::LandmarksDetectionResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the person attributes detection result.
   * @param[in] A person attributes detection result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::PersonAttribsDetectionResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the person reidentification result.
   * @param[in] A person reidentification result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::PersonReidentificationResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the face detection result.
   * @param[in] A face detection result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::FaceDetectionResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the object detection result.
   * @param[in] results A bundle of object detection results.
   */
  void accept(const std::vector<dynamic_vino_lib::ObjectDetectionResult> &) override;
  /**
 * @brief Generate rviz output content according to
 * the object segmentation result.
 * @param[in] results A bundle of object segmentation results.
 */
  void accept(const std::vector<dynamic_vino_lib::ObjectSegmentationResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the emotion detection result.
   * @param[in] A emotion detection result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::EmotionsResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the age and gender detection result.
   * @param[in] A head pose detection result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::HeadPoseResult> &) override;
  /**
   * @brief Generate rviz output content according to
   * the headpose detection result.
   * @param[in] An age gender detection result objetc.
   */
  void accept(const std::vector<dynamic_vino_lib::AgeGenderResult> &) override;

private:
  std_msgs::msg::Header getHeader();
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  std::shared_ptr<sensor_msgs::msg::Image> image_topic_;
  std::shared_ptr<Outputs::ImageWindowOutput> image_window_output_;
};
}  // namespace Outputs
#endif  // DYNAMIC_VINO_LIB__OUTPUTS__RVIZ_OUTPUT_HPP_
