#include "rclcpp/rclcpp.hpp"
#include "openvino/viz_factory.hpp"
#include "openvino/viz_base.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("infer_node");

  std::string infer_type = node.declare_parameter("infer_type").get<rclcpp::PARAMETER_STRING>();;
  auto viz_factory = std::make_shared<openvino::VizFactory>();
  auto viz_node = viz_factory->createVizNode(infer_type, "detection_viz", "/", rclcpp::NodeOptions());
  viz_node->composeMessages();
  rclcpp::spin(viz_node);
}
