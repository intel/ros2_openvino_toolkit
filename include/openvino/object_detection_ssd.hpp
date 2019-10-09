#ifndef OPENVINO__OBJECT_DETECTION_SSD_HPP_
#define OPENVINO__OBJECT_DETECTION_SSD_HPP_

#include "rdk_interfaces/msg/objects_in_boxes.hpp"
#include "inference_engine.hpp"
#include "openvino_base.hpp"

namespace openvino
{
class ObjectDetectionSSD : public OpenVINOBase
{
public:
  ObjectDetectionSSD(rclcpp::Node & node);
  virtual ~ObjectDetectionSSD() = default;
  void initSubscriber() override;
  void initPublisher() override;
  void prepareInputBlobs() override;
  void prepareOutputBlobs() override;
  void process(const sensor_msgs::msg::Image::ConstSharedPtr msg) override;
  void process(const sensor_msgs::msg::Image::UniquePtr msg) override;
  void process(const sensor_msgs::msg::Image::ConstSharedPtr msg, const rdk_interfaces::msg::ObjectsInBoxes::ConstSharedPtr bboxes) override {};

  void process(cv::Mat & cv_image, rdk_interfaces::msg::ObjectsInBoxes & objs);

private:
  std::string input_name_;
  std::string output_name_;
  int max_proposal_count_;
  int object_size_;
  rclcpp::Publisher<rdk_interfaces::msg::ObjectsInBoxes>::SharedPtr pub_;
};
}  // namespace openvino

#endif  // OPENVINO__OBJECT_DETECTION_SSD_HPP_
