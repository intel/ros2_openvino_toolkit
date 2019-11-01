#ifndef OPENVINO__OBJECT_SEGMENTATION_HPP_
#define OPENVINO__OBJECT_SEGMENTATION_HPP_

#include "rdk_interfaces/msg/objects_in_boxes.hpp"
#include "rdk_interfaces/msg/objects_in_masks.hpp"
#include "inference_engine.hpp"
#include "openvino_base.hpp"

namespace openvino
{
class ObjectSegmentation : public OpenVINOBase
{
public:
  ObjectSegmentation(rclcpp::Node & node);
  virtual ~ObjectSegmentation() = default;
  void initSubscriber() override;
  void initPublisher() override;
  void prepareInputBlobs() override;
  void prepareOutputBlobs() override;
  template <typename T>
  void process(const T msg);
  void process(cv::Mat & cv_image, rdk_interfaces::msg::ObjectsInMasks & objs);

private:
  std::string input_info_name_;
  std::string input_tensor_name_;
  std::string detection_output_name_;
  std::string mask_output_name_;
  int object_size_;
  rclcpp::Publisher<rdk_interfaces::msg::ObjectsInMasks>::SharedPtr pub_;
};
}  // namespace openvino

#endif  // OPENVINO__OBJECT_SEGMENTATION_HPP_
