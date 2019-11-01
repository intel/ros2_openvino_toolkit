#include "openvino/openvino_factory.hpp"
#include "openvino/object_detection_ssd.hpp"
// #include "openvino/object_detection_yolov2.hpp"
// #include "openvino/object_segmentation.hpp"
// #include "openvino/reidentification.hpp"

namespace openvino
{
OpenVINOFactory::OpenVINOFactory(const rclcpp::NodeOptions & node_options)
: Node("openvino", "/", node_options)
{
  init();
}

OpenVINOFactory::OpenVINOFactory(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: Node(node_name, ns, node_options)
{
  init();
}

void OpenVINOFactory::init()
{
  std::string model_type = declare_parameter("type").get<rclcpp::PARAMETER_STRING>();
  auto iter = MODEL_MAP.find(model_type);
  if (iter == MODEL_MAP.end()) {
    RCLCPP_INFO(get_logger(), "Unsupported model type: %s", model_type.c_str());
    printSupportedModelType();
    rclcpp::shutdown();
  } else {
    switch(MODEL_MAP.at(model_type))
    {
      case SSD:
        RCLCPP_INFO(get_logger(), "SSD");
        ov_node_ = std::make_shared<ObjectDetectionSSD>(*this);
        break;
      // case YOLOV2:
      //   RCLCPP_INFO(get_logger(), "YOLOV2");
      //   ov_node_ = std::make_shared<ObjectDetectionYOLOV2>(*this);
      //   break;
      // case ReID:
      //   RCLCPP_INFO(get_logger(), "ReID");
      //   ov_node_ = std::make_shared<Reidentification>(*this);
      //   break;
      // case Segment:
      //   RCLCPP_INFO(get_logger(), "Segmentation");
      //   ov_node_ = std::make_shared<ObjectSegmentation>(*this);
      //   break;
    }
  }  
}

void OpenVINOFactory::printSupportedModelType()
{
  int cnt = 0;
  RCLCPP_INFO(get_logger(), "Current supported CNN model:");
  for (auto iter = MODEL_MAP.begin(); iter != MODEL_MAP.end(); iter++) {
    RCLCPP_INFO(get_logger(), "%d: %s", cnt, iter->first.c_str());
    cnt++;
  }
}
}  // namespace openvino

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(openvino::OpenVINOFactory)