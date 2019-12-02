#include "rclcpp/rclcpp.hpp"
#include "openvino/viz_factory.hpp"
#include "openvino/viz_base.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  std::string type = "Detection";
  auto viz_factory = std::make_shared<openvino::VizFactory>();
  auto viz_node = viz_factory->createVizNode(type, "detection_viz", "/", rclcpp::NodeOptions());
  viz_node->composeMessages();
  rclcpp::spin(viz_node);
}
