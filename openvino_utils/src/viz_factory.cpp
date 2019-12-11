#include "openvino/viz_factory.hpp"
#include "openvino/viz_detection.hpp"
#include "openvino/viz_segmentation.hpp"

namespace openvino
{
std::shared_ptr<openvino::VizBase> VizFactory::createVizNode(const std::string & type, const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & node_options)
{
  switch(Infer_MAP.at(type))
  {
    case SSD:
      return std::make_shared<VizDetection>(node_name, ns, node_options);
      break;
    case Segmentation:
      return std::make_shared<VizSegmentation>(node_name, ns, node_options);
      break;
    case ReID:
      break;
  }
}
}
