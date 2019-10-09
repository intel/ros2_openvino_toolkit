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
  OpenVINOFactory(const std::string & node_name, bool is_intra_process);
  virtual ~OpenVINOFactory() = default;
  void printSupportedModelType();
private:
  std::shared_ptr<OpenVINOBase> ov_node_;
  rclcpp::Logger logger_;
};
}  // namespace openvino
#endif  // OPENVINO__OPENVINO_FACTORY_HPP_
