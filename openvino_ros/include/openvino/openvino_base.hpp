#ifndef OPENVINO__OPENVINO_BASE_HPP_
#define OPENVINO__OPENVINO_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "inference_engine.hpp"
#include "object_msgs/msg/objects_in_boxes.hpp"
#include "opencv2/opencv.hpp"

using namespace InferenceEngine;

namespace openvino
{
class OpenVINOBase
{
public:
  OpenVINOBase(rclcpp::Node & node);
  virtual ~OpenVINOBase() = default;
  void init();
  void loadEngine();
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
  virtual void registerInferCompletionCallback() = 0;

protected:
  rclcpp::Node & node_;
  std::vector<std::string> labels_;
  Core ie_core_;
  std::string device_name_;
  CNNNetwork network_;
  ExecutableNetwork exec_network_;
  InferRequest::Ptr async_infer_request_;
  bool is_first_frame_;
  size_t num_channels_;
  size_t blob_width_;
  size_t blob_height_;
  size_t cv_width_;
  size_t cv_height_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
}  // namespace openvino
#endif  // OPENVINO__OPENVINO_BASE_HPP_
