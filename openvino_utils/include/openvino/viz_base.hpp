#ifndef OPENVINO__VIZ_BASE_HPP_
#define OPENVINO__VIZ_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace openvino
{
class VizBase : public rclcpp::Node
{
public:
  VizBase(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
  : Node(node_name, ns, node_options)
  {};
  virtual ~VizBase() = default;
  virtual void composeMessages() = 0;
};
}  // namespace openvino

#endif // OPENVINO__VIZ_BASE_HPP_