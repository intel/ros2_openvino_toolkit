#ifndef OPENVINO__VISUALIZATION_HPP_
#define OPENVINO__VISUALIZATION_HPP_

#include "rclcpp/rclcpp.hpp"

namespace openvino
{
class Visualization : public rclcpp::Node
{
public:
  Visualization(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
  ~Visualization();
  void compose(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectInBoxes::ConstSharedPtr objs_msg);
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
}  // namespace openvino

#endif  // OPENVINO__VISUALIZATION_HPP_
