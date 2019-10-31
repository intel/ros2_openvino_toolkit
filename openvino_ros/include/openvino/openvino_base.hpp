#ifndef OPENVINO__OPENVINO_BASE_HPP_
#define OPENVINO__OPENVINO_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "inference_engine.hpp"
#include "rdk_interfaces/msg/objects_in_boxes.hpp"
#include "opencv2/opencv.hpp"

using namespace InferenceEngine;

const std::string PLUGIN_DIRS = "/opt/intel/openvino/deployment_tools/inference_engine/lib/intel64";

namespace openvino
{
class OpenVINOBase
{
public:
  OpenVINOBase(rclcpp::Node & node);
  virtual ~OpenVINOBase() = default;
  void init();
  void loadEngine(const std::string & engine_name);
  void readNetwork(const std::string & model_path,
  const std::string & weights_path);
  void readLabels(const std::string & label_path);
  void loadNetwork();

  inline CNNNetwork & getNetwork()
  {
    return network_;
  }

  inline std::vector<std::string> & getLabels()
  {
    return labels_;
  }

  inline ExecutableNetwork & getExecNetwork()
  {
    return exec_network_;
  }

  virtual void initSubscriber() = 0;
  virtual void initPublisher() = 0;
  virtual void prepareInputBlobs() = 0;
  virtual void prepareOutputBlobs() = 0;

protected:
  rclcpp::Node & node_;
  std::vector<std::string> labels_;
  InferencePlugin plugin_;
  CNNNetwork network_;
  ExecutableNetwork exec_network_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
}  // namespace openvino
#endif  // OPENVINO__OPENVINO_BASE_HPP_
