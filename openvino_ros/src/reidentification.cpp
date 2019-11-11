#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "object_msgs/msg/object_in_box.hpp"
#include "object_msgs/msg/object.hpp"
#include "openvino/reidentification.hpp"

using namespace InferenceEngine;

namespace openvino
{
Reidentification::Reidentification(rclcpp::Node & node)
: OpenVINOBase(node)
{
  double confidence_threshold = node_.declare_parameter("confidence_threshold").get<rclcpp::PARAMETER_DOUBLE>();

  tracker_ = std::make_shared<Tracker>(1000, confidence_threshold, 0.3);
  init();
}

void Reidentification::prepareInputBlobs()
{
  InputsDataMap input_info(network_.getInputsInfo());
  if (input_info.size() != 1) {
    RCLCPP_ERROR(node_.get_logger(), "Re-identification network has only one input");
    rclcpp::shutdown();
  }
  InputInfo::Ptr & input = input_info.begin()->second;
  input_name_ = input_info.begin()->first;

  input->setPrecision(Precision::U8);
  input->getInputData()->setLayout(Layout::NCHW);
}

void Reidentification::prepareOutputBlobs()
{
  OutputsDataMap output_info(network_.getOutputsInfo());

  DataPtr & output = output_info.begin()->second;
  output_name_ = output_info.begin()->first;
  
  output->setPrecision(Precision::FP32);
  output->setLayout(Layout::NCHW);
}

void Reidentification::initSubscriber()
{

  std::string input_topic1 = node_.declare_parameter("input_topic1").get<rclcpp::PARAMETER_STRING>();
  std::string input_topic2 = node_.declare_parameter("input_topic2").get<rclcpp::PARAMETER_STRING>();

  cam_sub_ = std::make_unique<CamSub>(&node_, input_topic1);
  obj_sub_ = std::make_unique<ObjSub>(&node_, input_topic2);
  sync_sub_ = std::make_unique<Sync>(*cam_sub_, *obj_sub_, 10);
  sync_sub_->registerCallback(std::bind(&Reidentification::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void Reidentification::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const object_msgs::msg::ObjectsInBoxes::ConstSharedPtr bboxes)
{
  cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uchar *>(&msg->data[0]),
    msg->step);
  for(unsigned int i = 0; i < bboxes->objects_vector.size(); i++)
  {
    cv::Rect roi(bboxes->objects_vector[i].roi.x_offset, bboxes->objects_vector[i].roi.y_offset, bboxes->objects_vector[i].roi.width, bboxes->objects_vector[i].roi.height);
    cv::Mat cropped_image = cv_image(roi);

    reid_.header = msg->header;
    process(cropped_image);
  }
}

void Reidentification::initPublisher()
{
  std::string output_topic = node_.declare_parameter("output_topic").get<rclcpp::PARAMETER_STRING>();
  pub_ = node_.create_publisher<object_msgs::msg::ReidentificationStamped>(output_topic, 16);
}

void Reidentification::process(cv::Mat & cv_image)
{
  if (is_first_frame_ == true) {
    is_first_frame_ = false;
  } else {
    async_infer_request_->Wait(IInferRequest::WaitMode::RESULT_READY);
  }

  Blob::Ptr image_input = async_infer_request_->GetBlob(input_name_);

  num_channels_ = image_input->getTensorDesc().getDims()[1];
  blob_width_ = image_input->getTensorDesc().getDims()[3];
  blob_height_ = image_input->getTensorDesc().getDims()[2];

  unsigned char* blob_data = static_cast<unsigned char*>(image_input->buffer());
  int cv_width = cv_image.cols;
  int cv_height = cv_image.rows;

  cv::Mat resized_image(cv_image);
  if (blob_width_ != cv_width || blob_height_ != cv_height) {
    cv::resize(cv_image, resized_image, cv::Size(blob_width_, blob_height_));
  }

  for (size_t c = 0; c < num_channels_; c++) {
    for (size_t h = 0; h < blob_height_; h++) {
      for (size_t w = 0; w < blob_width_; w++) {
        blob_data[c * blob_width_ * blob_height_ + h * blob_width_ + w] =
          resized_image.at<cv::Vec3b>(h, w)[c];
      }
    }
  }

  async_infer_request_->StartAsync();
}

void Reidentification::registerInferCompletionCallback()
{
  async_infer_request_ = exec_network_.CreateInferRequestPtr();

  auto callback = [&] {
    reid_.reidentified_vector.clear();
    const float * output_values = async_infer_request_->GetBlob(output_name_)->buffer().as<float *>();
    std::vector<float> new_item = std::vector<float>(output_values, output_values + 256);
    std::string item_id = "No." + std::to_string(tracker_->processNewTracker(new_item));
    object_msgs::msg::Reidentification reid;
    reid.identity = item_id;
    reid_.reidentified_vector.push_back(reid);
    pub_->publish(reid_);
  };

  async_infer_request_->SetCompletionCallback(callback);
}

}  // namespace openvino
