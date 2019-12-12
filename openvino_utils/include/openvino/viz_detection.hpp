#ifndef OPENVINO__VIZ_DETECTION_HPP_
#define OPENVINO__VIZ_DETECTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"
#include "object_msgs/msg/objects_in_boxes.hpp"
#include "object_msgs/msg/object_in_box.hpp"
#include "openvino/viz_base.hpp"

namespace openvino
{
class VizDetection : public VizBase
{
public:
  using CamSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using ObjSub = message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes>;
  using Sync = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, object_msgs::msg::ObjectsInBoxes>;

  VizDetection(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options);
  virtual ~VizDetection() = default;
  void composeMessages() override;
  
private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr objs_msg);

  std::unique_ptr<CamSub> cam_sub_;
  std::unique_ptr<ObjSub> obj_sub_;
  std::unique_ptr<Sync> sync_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
}

#endif  // OPENVINO__VIZ_DETECTION_HPP_
