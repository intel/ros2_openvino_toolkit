#ifndef OPENVINO__OBJECT_DETECTION_SSD_HPP_
#define OPENVINO__OBJECT_DETECTION_SSD_HPP_

#include "object_msgs/msg/objects_in_boxes.hpp"
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
  void registerInferCompletionCallback() override;

  template <typename T>
  void process(const T msg);

private:
  std::string input_name_;
  std::string output_name_;
  int max_proposal_count_;
  int object_size_;
  object_msgs::msg::ObjectsInBoxes objs_;
  rclcpp::Publisher<object_msgs::msg::ObjectsInBoxes>::SharedPtr pub_;
};
}  // namespace openvino

#endif  // OPENVINO__OBJECT_DETECTION_SSD_HPP_
