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
 * @brief A header file with declaration for HeadPoseDetectionModel Class
 * @file head_pose_detection_model.h
 */

#ifndef DYNAMIC_VINO_LIB__OUTPUTS__BASE_OUTPUT_HPP_
#define DYNAMIC_VINO_LIB__OUTPUTS__BASE_OUTPUT_HPP_

#include <object_msgs/msg/object_in_box.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <people_msgs/msg/emotion.hpp>
#include <people_msgs/msg/emotions_stamped.hpp>
#include <people_msgs/msg/age_gender.hpp>
#include <people_msgs/msg/age_gender_stamped.hpp>
#include <people_msgs/msg/head_pose.hpp>
#include <people_msgs/msg/head_pose_stamped.hpp>
#include <people_msgs/srv/age_gender.hpp>
#include <people_msgs/srv/emotion.hpp>
#include <people_msgs/srv/people.hpp>
#include <people_msgs/srv/head_pose.hpp>
#include <object_msgs/srv/detect_object.hpp>
#include <string>
#include <vector>
#include <memory>

#include "dynamic_vino_lib/inferences/age_gender_detection.hpp"
#include "dynamic_vino_lib/inferences/base_inference.hpp"
#include "dynamic_vino_lib/inferences/emotions_detection.hpp"
#include "dynamic_vino_lib/inferences/face_detection.hpp"
#include "dynamic_vino_lib/inferences/head_pose_detection.hpp"
#include "dynamic_vino_lib/inferences/object_detection.hpp"
#include "dynamic_vino_lib/inferences/object_segmentation.hpp"
#include "dynamic_vino_lib/inferences/person_reidentification.hpp"
#include "dynamic_vino_lib/inferences/person_attribs_detection.hpp"
#include "dynamic_vino_lib/inferences/landmarks_detection.hpp"
#include "dynamic_vino_lib/inferences/face_reidentification.hpp"
#include "dynamic_vino_lib/inferences/vehicle_attribs_detection.hpp"
#include "dynamic_vino_lib/inferences/license_plate_detection.hpp"
#include "opencv2/opencv.hpp"

class Pipeline;
namespace Outputs
{
/**
 * @class BaseOutput
 * @brief This class is a base class for various output devices. It employs
 * visitor pattern to perform different operations to different inference
 * result with different output device
 */
class BaseOutput
{
public:
  BaseOutput() = default;
  /**
   * @brief Generate output content according to the license plate detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::LicensePlateDetectionResult> &)
  {
  }
  /**
   * @brief Generate output content according to the vehicle attributes detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::VehicleAttribsDetectionResult> &)
  {
  }
  /**
   * @brief Generate output content according to the face reidentification result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::FaceReidentificationResult> &)
  {
  }
  /**
   * @brief Generate output content according to the landmarks detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::LandmarksDetectionResult> &)
  {
  }
  /**
   * @brief Generate output content according to the person reidentification result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::PersonAttribsDetectionResult> &)
  {
  }
  /**
   * @brief Generate output content according to the person reidentification result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::PersonReidentificationResult> &)
  {
  }
  /**
   * @brief Generate output content according to the object segmentation result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::ObjectSegmentationResult> &)
  {
  }
  /**
   * @brief Generate output content according to the object detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::ObjectDetectionResult> &)
  {
  }
  /**
   * @brief Generate output content according to the face detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::FaceDetectionResult> &)
  {
  }
  /**
   * @brief Generate output content according to the emotion detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::EmotionsResult> &)
  {
  }
  /**
   * @brief Generate output content according to the age and gender detection
   * result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::AgeGenderResult> &)
  {
  }
  /**
   * @brief Generate output content according to the headpose detection result.
   */
  virtual void accept(const std::vector<dynamic_vino_lib::HeadPoseResult> &)
  {
  }
  /**
   * @brief Calculate the camera matrix of a frame for image window output, no
         implementation for ros topic output.
   */
  virtual void feedFrame(const cv::Mat &)
  {
  }
  /**
   * @brief Show all the contents generated by the accept functions.
   */
  virtual void handleOutput() = 0;

  void setPipeline(Pipeline * const pipeline);
  virtual void setServiceResponse(
    std::shared_ptr<object_msgs::srv::DetectObject::Response> response) {}
  virtual void setServiceResponseForFace(
    std::shared_ptr<object_msgs::srv::DetectObject::Response> response) {}
  virtual void setServiceResponse(
    std::shared_ptr<people_msgs::srv::AgeGender::Response> response) {}
  virtual void setServiceResponse(
    std::shared_ptr<people_msgs::srv::Emotion::Response> response) {}
  virtual void setServiceResponse(
    std::shared_ptr<people_msgs::srv::HeadPose::Response> response) {}
  virtual void setServiceResponse(
    std::shared_ptr<people_msgs::srv::People::Response> response) {}
  Pipeline * getPipeline() const;
  cv::Mat getFrame() const;
  virtual void clearData() {}

protected:
  cv::Mat frame_;
  Pipeline * pipeline_;
};
}  // namespace Outputs
#endif  // DYNAMIC_VINO_LIB__OUTPUTS__BASE_OUTPUT_HPP_
