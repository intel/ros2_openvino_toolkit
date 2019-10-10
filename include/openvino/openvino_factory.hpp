#ifndef OPENVINO__OPENVINO_FACTORY_HPP_
#define OPENVINO__OPENVINO_FACTORY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "openvino_base.hpp"

namespace openvino
{
enum Model
{
  SSD,
  YOLOV2,
  ReID,
  Segment
};

const std::map<std::string, Model> MODEL_MAP
  = {{"SSD", SSD},
     {"YOLOV2", YOLOV2},
     {"ReID", ReID},
     {"Segment", Segment}};

class OpenVINOFactory : public rclcpp::Node
{
public:
  OpenVINOFactory(const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
  OpenVINOFactory(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options=rclcpp::NodeOptions());
  virtual ~OpenVINOFactory() = default;
  void printSupportedModelType();
private:
  void init();
  std::shared_ptr<OpenVINOBase> ov_node_;
};
}  // namespace openvino
#endif  // OPENVINO__OPENVINO_FACTORY_HPP_
