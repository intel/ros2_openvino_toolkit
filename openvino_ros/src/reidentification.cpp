#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "rdk_interfaces/msg/object_in_box.hpp"
#include "rdk_interfaces/msg/object.hpp"
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

void Reidentification::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const rdk_interfaces::msg::ObjectsInBoxes::ConstSharedPtr bboxes)
{
  process(msg, bboxes);
}

void Reidentification::initPublisher()
{
  std::string output_topic = node_.declare_parameter("output_topic").get<rclcpp::PARAMETER_STRING>();
  pub_ = node_.create_publisher<rdk_interfaces::msg::Reidentification>(output_topic, 16);
}

void Reidentification::process(const sensor_msgs::msg::Image::ConstSharedPtr msg, const rdk_interfaces::msg::ObjectsInBoxes::ConstSharedPtr bboxes)
{
  cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uchar *>(&msg->data[0]),
    msg->step);
  for(unsigned int i = 0; i < bboxes->objects_vector.size(); i++)
  {
    cv::Rect roi(bboxes->objects_vector[i].roi.x_offset, bboxes->objects_vector[i].roi.y_offset, bboxes->objects_vector[i].roi.width, bboxes->objects_vector[i].roi.height);
    cv::Mat cropped_image = cv_image(roi);

    rdk_interfaces::msg::Reidentification reid;
    reid.header = msg->header;
    process(cropped_image, reid);
    pub_->publish(reid);
  }
}

void Reidentification::process(cv::Mat & cv_image, rdk_interfaces::msg::Reidentification & reid)
{
  InferRequest::Ptr async_infer_request = exec_network_.CreateInferRequestPtr();
  Blob::Ptr image_input = async_infer_request->GetBlob(input_name_);

  size_t num_channels = image_input->getTensorDesc().getDims()[1];
  size_t blob_width = image_input->getTensorDesc().getDims()[3];
  size_t blob_height = image_input->getTensorDesc().getDims()[2];

  unsigned char* blob_data = static_cast<unsigned char*>(image_input->buffer());
  int cv_width = cv_image.cols;
  int cv_height = cv_image.rows;

  cv::Mat resized_image(cv_image);
  if (blob_width != cv_width || blob_height != cv_height) {
    cv::resize(cv_image, resized_image, cv::Size(blob_width, blob_height));
  }

  for (size_t c = 0; c < num_channels; c++) {
    for (size_t h = 0; h < blob_height; h++) {
      for (size_t w = 0; w < blob_width; w++) {
        blob_data[c * blob_width * blob_height + h * blob_width + w] =
          resized_image.at<cv::Vec3b>(h, w)[c];
      }
    }
  }

  async_infer_request->StartAsync();
  async_infer_request->Wait(IInferRequest::WaitMode::RESULT_READY);

  const float * output_values = async_infer_request->GetBlob(output_name_)->buffer().as<float *>();

  std::vector<float> new_item = std::vector<float>(
    output_values, output_values + 256);
  std::string item_id = "No." + std::to_string(
    tracker_->processNewTracker(new_item));
  reid.identity = item_id;
}
}  // namespace openvino
