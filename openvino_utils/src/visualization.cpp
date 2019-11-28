#include <iostream>
#include <iomanip>
#include <memory>
#include <cv_bridge/cv_bridge.h>

#include "visualization.hpp"

namespace openvino
{
Visualization::Visualization(const rclcpp::NodeOptions & node_options)
: Node("openvino_util", "/", node_options)
{
}

Visualization::Visualization(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: Node(node_name, ns , node_options)
{
}

void Visualization::mergeDetectionMsgs()
{
  using CamSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using ObjSub = message_filters::Subscriber<object_msgs::msg::ObjectsInBoxes>;
  using Sync = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, object_msgs::msg::ObjectsInBoxes>;

  std::unique_ptr<CamSub> cam_sub = std::make_unique<CamSub>(this, "/camera/color/image_raw");
  std::unique_ptr<ObjSub> obj_sub = std::make_unique<ObjSub>(this, "/openvino/detected_objects");
  std::unique_ptr<Sync> sync_sub = std::make_unique<Sync>(*cam_sub, *obj_sub, 10);
  sync_sub->registerCallback(std::bind(&Visualization::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void Visualization::callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr objs_msg)
{
  cv::Mat cv_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  for (unsigned int i = 0; i < objs_msg->objects_vector.size(); i++)
  {
    float xmin = objs_msg->objects_vector[i].roi.x_offset;
    float ymin = objs_msg->objects_vector[i].roi.y_offset;
    float xmax = objs_msg->objects_vector[i].roi.x_offset + objs_msg->objects_vector[i].roi.width;
    float ymax = objs_msg->objects_vector[i].roi.y_offset + objs_msg->objects_vector[i].roi.height;

    std::ostringstream conf;
    conf << ":" << std::fixed << std::setprecision(3) << objs_msg->objects_vector[i].object.probability;

    cv::putText(cv_image, objs_msg->objects_vector[i].object.object_name + conf.str(),
      cv::Point2f(xmin, ymin - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));

    cv::rectangle(cv_image, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), cv::Scalar(0, 0, 255));
  }

  std::shared_ptr<cv_bridge::CvImage> cv_ptr =
    std::make_shared<cv_bridge::CvImage>(image_msg->header, "bgr8", cv_image);
  std::shared_ptr<sensor_msgs::msg::Image> image_topic = cv_ptr->toImageMsg();
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rdk/openvino/detection_inboxes", 16);
  pub_->publish(*image_topic);
}

void Visualization::mergeSegmentationMsgs()
{

}

void Visualization::mergeReidentificationMsgs()
{

}
}  // namespace openvino

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using namespace openvino;
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

  auto viz = std::make_shared<Visualization>("visualization", "/", options);

  std::string inference_type = "Detection";

  switch(Infer_MAP.at(inference_type))
  {
    case Detection:
      viz->mergeDetectionMsgs();
      break;
    case Segmentation:
      viz->mergeSegmentationMsgs();
      break;
    case ReID:
      viz->mergeReidentificationMsgs();
      break;
  }
  rclcpp::spin(viz);
}
