#ifndef OPENVINO__VISUALIZATION_HPP_
#define OPENVINO__VISUALIZATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "object_msgs/msg/objects_in_boxes.hpp"
#include "object_msgs/msg/object_in_box.hpp"

namespace openvino
{
enum InferType
{
  Detection,
  Segmentation,
  ReID
};

const std::map<std::string, InferType> Infer_MAP
  = {{"Detection", Detection},
     {"Segmentation", Segmentation},
     {"ReID", ReID}};

class Visualization : public rclcpp::Node
{
public:
  Visualization(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
  Visualization(const std::string & node_name, const std::string & ns, 
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ~Visualization() = default;
  void mergeDetectionMsgs();
  void mergeSegmentationMsgs();
  void mergeReidentificationMsgs();

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr objs_msg);

};
}  // namespace openvino

#endif  // OPENVINO__VISUALIZATION_HPP_
