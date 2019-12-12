#ifndef OPENVINO__VIZ_REIDENTIFICATION_HPP_
#define OPENVINO__VIZ_REIDENTIFICATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"
#include "object_msgs/msg/reidentification.hpp"
#include "object_msgs/msg/reidentification_stamped.hpp"
#include "openvino/viz_base.hpp"

namespace openvino
{
class VizReidentification : public VizBase
{
public:
  using CamSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using ReidSub = message_filters::Subscriber<object_msgs::msg::ReidentificationStamped>;
  using Sync = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, object_msgs::msg::ReidentificationStamped>;

  VizReidentification(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options);
  virtual ~VizReidentification() = default;
  void composeMessages() override;
  
private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ReidentificationStamped::ConstSharedPtr objs_msg);

  std::unique_ptr<CamSub> cam_sub_;
  std::unique_ptr<ReidSub> reid_sub_;
  std::unique_ptr<Sync> sync_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
}

#endif  // OPENVINO__VIZ_REIDENTIFICATION_HPP_
