#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <memory>

#include "openvino/viz_reidentification.hpp"

namespace openvino
{
VizReidentification::VizReidentification(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: VizBase(node_name, ns, node_options)
{
}

void VizReidentification::composeMessages()
{
  cam_sub_ = std::make_unique<CamSub>(this, "/rdk/openvino/image_raw");
  reid_sub_ = std::make_unique<ReidSub>(this, "/rdk/openvino/object_id");
  sync_sub_ = std::make_unique<Sync>(*cam_sub_, *reid_sub_, 10);
  sync_sub_->registerCallback(std::bind(&VizReidentification::callback, this, std::placeholders::_1, std::placeholders::_2));

  pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rdk/openvino/reidentification", 16);
}

void VizReidentification::callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ReidentificationStamped::ConstSharedPtr objs_msg)
{
  cv::Mat cv_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  for (unsigned int i = 0; i < objs_msg->reidentified_vector.size(); i++)
  {
    float xmin = objs_msg->reidentified_vector[i].roi.x_offset;
    float ymin = objs_msg->reidentified_vector[i].roi.y_offset;
    float xmax = objs_msg->reidentified_vector[i].roi.x_offset + objs_msg->reidentified_vector[i].roi.width;
    float ymax = objs_msg->reidentified_vector[i].roi.y_offset + objs_msg->reidentified_vector[i].roi.height;

    cv::putText(cv_image, objs_msg->reidentified_vector[i].identity, cv::Point2f(xmin, ymin - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));

    cv::rectangle(cv_image, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), cv::Scalar(0, 0, 255));
  }

  std::shared_ptr<cv_bridge::CvImage> cv_ptr =
    std::make_shared<cv_bridge::CvImage>(image_msg->header, "bgr8", cv_image);
  std::shared_ptr<sensor_msgs::msg::Image> image_topic = cv_ptr->toImageMsg();
  pub_->publish(*image_topic);
}
}  // namespace openvino
