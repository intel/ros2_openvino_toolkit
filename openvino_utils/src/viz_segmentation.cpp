#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <memory>

#include <cstring>

#include "openvino/viz_segmentation.hpp"

namespace openvino
{
VizSegmentation::VizSegmentation(const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & node_options)
: VizBase(node_name, ns, node_options)
{
}

void VizSegmentation::composeMessages()
{
  cam_sub_ = std::make_unique<CamSub>(this, "/rdk/openvino/image_raw");
  mask_sub_ = std::make_unique<MaskSub>(this, "/rdk/openvino/segmented_objects");
  sync_sub_ = std::make_unique<Sync>(*cam_sub_, *mask_sub_, 50);
  sync_sub_->registerCallback(std::bind(&VizSegmentation::callback, this, std::placeholders::_1, std::placeholders::_2));

  pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rdk/openvino/segmented_with_mask", 16);
}

void VizSegmentation::callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const object_msgs::msg::ObjectsInMasks::ConstSharedPtr masks_msg)
{
  cv::Mat cv_image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

  std::vector<std::vector<short>> colors = {
      {128, 64,  128},
      {232, 35,  244},
      {70,  70,  70},
      {156, 102, 102},
      {153, 153, 190},
      {153, 153, 153},
      {30,  170, 250},
      {0,   220, 220},
      {35,  142, 107},
      {152, 251, 152},
      {180, 130, 70},
      {60,  20,  220},
      {0,   0,   255},
      {142, 0,   0},
      {70,  0,   0},
      {100, 60,  0},
      {90,  0,   0},
      {230, 0,   0},
      {32,  11,  119},
      {0,   74,  111},
      {81,  0,   81}
  };
  std::map<std::string, int> class_color;
  const float MASK_THRESHOLD = 0.7f;
  const float alpha = 0.7f;

  for (unsigned int i = 0; i < masks_msg->objects_vector.size(); i++)
  {
    std::string class_label = masks_msg->objects_vector[i].object_name;
    if (class_color.find(class_label) == class_color.end()) {
      class_color[class_label] = class_color.size();
    }
    auto & color = colors[class_color[class_label]];

    cv::Rect roi;
    roi.x = masks_msg->objects_vector[i].roi.x_offset;
    roi.y = masks_msg->objects_vector[i].roi.y_offset;
    roi.width = masks_msg->objects_vector[i].roi.width;
    roi.height = masks_msg->objects_vector[i].roi.height;

    cv::Mat roi_input_img = cv_image(roi);
    cv::Mat mask_mat(roi.height, roi.width, CV_32FC1);

    for (int j = 0; j < roi.height; j++)
      for (int k = 0; k < roi.width; k++)
        mask_mat.at<float>(j,k) = masks_msg->objects_vector[i].mask_array[j * roi.width + k];

    cv::Mat uchar_resized_mask(roi.height, roi.width, cv_image.type());
    
    for (int h = 0; h < mask_mat.size().height; ++h)
      for (int w = 0; w < mask_mat.size().width; ++w)
        for (int ch = 0; ch < uchar_resized_mask.channels(); ++ch)
          uchar_resized_mask.at<cv::Vec3b>(h, w)[ch] = mask_mat.at<float>(h, w) > MASK_THRESHOLD ? 255 * color[ch]: roi_input_img.at<cv::Vec3b>(h, w)[ch];

    cv::addWeighted(uchar_resized_mask, alpha, roi_input_img, 1.0f - alpha, 0.0f, roi_input_img);
  }

  std::shared_ptr<cv_bridge::CvImage> cv_ptr =
    std::make_shared<cv_bridge::CvImage>(image_msg->header, "bgr8", cv_image);
  std::shared_ptr<sensor_msgs::msg::Image> image_topic = cv_ptr->toImageMsg();

  pub_->publish(*image_topic);
}
}  // namespace openvino
