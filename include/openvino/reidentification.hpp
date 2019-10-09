#ifndef OPENVINO__REIDENTIFICATION_HPP_
#define OPENVINO__REIDENTIFICATION_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "rdk_interfaces/msg/objects_in_boxes.hpp"
#include "rdk_interfaces/msg/reidentification.hpp"
#include "inference_engine.hpp"
#include "openvino_base.hpp"
#include "tracker.hpp"

namespace openvino
{

class Reidentification : public OpenVINOBase
{
public:
  Reidentification(rclcpp::Node & node);
  virtual ~Reidentification() = default;

  void initSubscriber() override;
  void initPublisher() override;
  void prepareInputBlobs() override;
  void prepareOutputBlobs() override;
  void process(const sensor_msgs::msg::Image::UniquePtr msg) override {};
  void process(const sensor_msgs::msg::Image::ConstSharedPtr msg) override {};
  void process(const sensor_msgs::msg::Image::ConstSharedPtr msg, const rdk_interfaces::msg::ObjectsInBoxes::ConstSharedPtr bboxes) override;

  void process(cv::Mat & cv_image, rdk_interfaces::msg::Reidentification & reid);

private:
  std::string input_name_;
  std::string output_name_;
  int max_proposal_count_;
  int object_size_;
  rclcpp::Publisher<rdk_interfaces::msg::Reidentification>::SharedPtr pub_;
  std::shared_ptr<Tracker> tracker_;

  using CamSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using ObjSub = message_filters::Subscriber<rdk_interfaces::msg::ObjectsInBoxes>;
  using Sync =
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, rdk_interfaces::msg::ObjectsInBoxes>;
  
  std::unique_ptr<CamSub> cam_sub_;
  std::unique_ptr<ObjSub> obj_sub_;
  std::unique_ptr<Sync> sync_sub_;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const rdk_interfaces::msg::ObjectsInBoxes::ConstSharedPtr bboxes);

};
}  // namespace openvino

#endif  // OPENVINO__REIDENTIFICATION_HPP_
