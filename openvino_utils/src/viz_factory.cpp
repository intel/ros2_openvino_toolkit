#include "openvino/viz_factory.hpp"
#include "openvino/viz_detection.hpp"

namespace openvino
{
std::shared_ptr<openvino::VizBase> VizFactory::createVizNode(const std::string & type, const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & node_options)
{
  switch(Infer_MAP.at(type))
  {
    case Detection:
      return std::make_shared<VizDetection>(node_name, ns, node_options);
      break;
    case Segmentation:
      break;
    case ReID:
      break;
  }
}
}