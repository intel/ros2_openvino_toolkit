#include <cv_bridge/cv_bridge.h>

#include "openvino/viz_detection.hpp"

namespace openvino
{
VizDetection::VizDetection(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: VizBase(node_name, ns, node_options)
{
}

void VizDetection::composeMessages()
{
  cam_sub_ = std::make_unique<CamSub>(this, "/camera/color/image_raw");
  obj_sub_ = std::make_unique<ObjSub>(this, "/openvino/detected_objects");
  sync_sub_ = std::make_unique<Sync>(*cam_sub_, *obj_sub_, 10);
  sync_sub_->registerCallback(std::bind(&VizDetection::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void VizDetection::callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr objs_msg)
{
  // cv::Mat cv_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  // for (unsigned int i = 0; i < objs_msg->objects_vector.size(); i++)
  // {
  //   float xmin = objs_msg->objects_vector[i].roi.x_offset;
  //   float ymin = objs_msg->objects_vector[i].roi.y_offset;
  //   float xmax = objs_msg->objects_vector[i].roi.x_offset + objs_msg->objects_vector[i].roi.width;
  //   float ymax = objs_msg->objects_vector[i].roi.y_offset + objs_msg->objects_vector[i].roi.height;

  //   std::ostringstream conf;
  //   conf << ":" << std::fixed << std::setprecision(3) << objs_msg->objects_vector[i].object.probability;

  //   cv::putText(cv_image, objs_msg->objects_vector[i].object.object_name + conf.str(),
  //     cv::Point2f(xmin, ymin - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));

  //   cv::rectangle(cv_image, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), cv::Scalar(0, 0, 255));
  // }

  // std::shared_ptr<cv_bridge::CvImage> cv_ptr =
  //   std::make_shared<cv_bridge::CvImage>(image_msg->header, "bgr8", cv_image);
  // std::shared_ptr<sensor_msgs::msg::Image> image_topic = cv_ptr->toImageMsg();
  // pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rdk/openvino/detection_inboxes", 16);
  // pub_->publish(*image_topic);
}
}  // namespace openvino
