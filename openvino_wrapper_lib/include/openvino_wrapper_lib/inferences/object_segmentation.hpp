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
 * @brief A header file with declaration for ObjectSegmentation Class
 * @file object_detection.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__OBJECT_SEGMENTATION_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__OBJECT_SEGMENTATION_HPP_
#include <object_msgs/msg/object.hpp>
#include <object_msgs/msg/object_in_box.hpp>
#include <object_msgs/msg/objects_in_boxes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>
#include "openvino_wrapper_lib/models/object_segmentation_model.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/inferences/base_inference.hpp"
#include "openvino/openvino.hpp"
#include "opencv2/opencv.hpp"
// namespace
namespace openvino_wrapper_lib
{
/**
 * @class ObjectSegmentationResult
 * @brief Class for storing and processing object segmentation result.
 */
class ObjectSegmentationResult : public Result
{
public:
  friend class ObjectSegmentation;
  explicit ObjectSegmentationResult(const cv::Rect & location);
  std::string getLabel() const
  {
    return label_;
  }
  /**
   * @brief Get the confidence that the detected area is a face.
   * @return The confidence value.
   */
  float getConfidence() const
  {
    return confidence_;
  }
  cv::Mat getMask() const
  {
    return mask_;
  }

private:
  std::string label_ = "";
  float confidence_ = -1;
  cv::Mat mask_;
};
/**
 * @class ObjectSegmentation
 * @brief Class to load object segmentation model and perform object segmentation.
 */
class ObjectSegmentation : public BaseInference
{
public:
  using Result = openvino_wrapper_lib::ObjectSegmentationResult;
  ObjectSegmentation() {};
  ~ObjectSegmentation() override {};

  /**
   * @brief initialize params
   */
  void init(const Params::ParamManager::InferenceRawData &val) override;
  /**
   * @brief Load the object segmentation model.
   */
  void loadNetwork(std::shared_ptr<Models::ObjectSegmentationModel>);
  /**
   * @brief Enqueue a frame to this class.
   * The frame will be buffered but not infered yet.
   * @param[in] frame The frame to be enqueued.
   * @param[in] input_frame_loc The location of the enqueued frame with respect
   * to the frame generated by the input device.
   * @return Whether this operation is successful.
   */
  bool enqueue(const cv::Mat &, const cv::Rect &) override;

  //Deprecated!!
  bool enqueue_for_one_input(const cv::Mat &, const cv::Rect &);

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
  const openvino_wrapper_lib::Result * getLocationResult(int idx) const override;
  /**
   * @brief Show the observed detection result either through image window
     or ROS topic.
   */
  void observeOutput(const std::shared_ptr<Outputs::BaseOutput> & output);
  /**
   * @brief Get the name of the Inference instance.
   * @return The name of the Inference instance.
   */
  const std::string getName() const override;
  const std::vector<cv::Rect> getFilteredROIs(
    const std::string filter_conditions) const override;

private:
  std::shared_ptr<Models::ObjectSegmentationModel> valid_model_;
  std::vector<Result> results_;
  int width_ = 0;
  int height_ = 0;
  double show_output_thresh_ = 0;

  std::vector<cv::Vec3b> colors_ = {
    {128, 64, 128}, {232, 35, 244}, {70, 70, 70}, {156, 102, 102}, {153, 153, 190},
    {153, 153, 153}, {30, 170, 250}, {0, 220, 220}, {35, 142, 107}, {152, 251, 152},
    {180, 130, 70}, {60, 20, 220}, {0, 0, 255}, {142, 0, 0}, {70, 0, 0},
    {100, 60, 0}, {90, 0, 0}, {230, 0, 0}, {32, 11, 119}, {0, 74, 111},
    {81, 0, 81}
  };
};
}  // namespace openvino_wrapper_lib
#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__OBJECT_SEGMENTATION_HPP_
