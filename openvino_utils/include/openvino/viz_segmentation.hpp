#ifndef OPENVINO__VIZ_SEGMENTATION_HPP_
#define OPENVINO__VIZ_SEGMENTATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"
#include "object_msgs/msg/objects_in_masks.hpp"
#include "object_msgs/msg/object_in_mask.hpp"
#include "openvino/viz_base.hpp"

namespace openvino
{
class VizSegmentation : public VizBase
{
public:
  using CamSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using MaskSub = message_filters::Subscriber<object_msgs::msg::ObjectsInMasks>;
  using Sync = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, object_msgs::msg::ObjectsInMasks>;

  VizSegmentation(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options);
  virtual ~VizSegmentation() = default;
  void composeMessages() override;
  
private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectsInMasks::ConstSharedPtr objs_msg);

  std::unique_ptr<CamSub> cam_sub_;
  std::unique_ptr<MaskSub> mask_sub_;
  std::unique_ptr<Sync> sync_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
}

#endif  // OPENVINO__VIZ_SEGMENTATION_HPP_
