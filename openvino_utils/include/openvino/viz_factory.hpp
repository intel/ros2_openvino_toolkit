#ifndef OPENVINO__VIZ_FACTORY_HPP_
#define OPENVINO__VIZ_FACTORY_HPP_

#include <map>
#include "openvino/viz_base.hpp"

namespace openvino
{
enum InferType
{
  SSD,
  Segmentation,
  ReID
};

const std::map<std::string, InferType> Infer_MAP
  = {{"SSD", SSD},
     {"Segmentation", Segmentation},
     {"ReID", ReID}};

class VizFactory
{
public:
  VizFactory() = default;
  virtual ~VizFactory() = default;
  std::shared_ptr<openvino::VizBase> createVizNode(const std::string & type, const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & node_options);
};
}  // namespace openvino

#endif  // OPENVINO__VIZ_FACTORY_HPP_
