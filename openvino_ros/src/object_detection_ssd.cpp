#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "rdk_interfaces/msg/object_in_box.hpp"
#include "rdk_interfaces/msg/object.hpp"
#include "openvino/object_detection_ssd.hpp"


using namespace InferenceEngine;

namespace openvino
{
ObjectDetectionSSD::ObjectDetectionSSD(rclcpp::Node & node)
: OpenVINOBase(node)
{
  init();
}

void ObjectDetectionSSD::prepareInputBlobs()
{
  InputsDataMap input_info(network_.getInputsInfo());
  if (input_info.size() != 1) {
  	RCLCPP_ERROR(node_.get_logger(), "SSD network has only one input");
    rclcpp::shutdown();
  }
  InputInfo::Ptr & input = input_info.begin()->second;
  input_name_ = input_info.begin()->first;

  input->setPrecision(Precision::U8);
  input->getInputData()->setLayout(Layout::NCHW);
}

void ObjectDetectionSSD::prepareOutputBlobs()
{
  OutputsDataMap output_info(network_.getOutputsInfo());

  DataPtr & output = output_info.begin()->second;
  output_name_ = output_info.begin()->first;

  const int num_classes = network_.getLayerByName(output_name_.c_str())->GetParamAsInt("num_classes");
  if (labels_.size() != num_classes) {
    if (labels_.size() == (num_classes - 1))  // if network assumes default "background" class, having no label
      labels_.insert(labels_.begin(), "fake");
    else
      labels_.clear();
  }
  const SizeVector output_dims = output->getTensorDesc().getDims();
  max_proposal_count_ = output_dims[2];
  object_size_ = output_dims[3];
  if (object_size_ != 7) {
    RCLCPP_ERROR(node_.get_logger(), "Output should have 7 as a last dimension");
  }
  if (output_dims.size() != 4) {
    RCLCPP_ERROR(node_.get_logger(), "Incorrect output dimensions for SSD");
  }
  output->setPrecision(Precision::FP32);
  output->setLayout(Layout::NCHW);
}

void ObjectDetectionSSD::initSubscriber()
{
  std::string input_topic = node_.declare_parameter("input_topic").get<rclcpp::PARAMETER_STRING>();

  if (!node_.get_node_options().use_intra_process_comms()) {
    auto callback = [this](sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
      process<sensor_msgs::msg::Image::ConstSharedPtr>(msg);
    };
    sub_ = node_.create_subscription<sensor_msgs::msg::Image>(input_topic, rclcpp::QoS(1), callback);
  } else {
    auto callback = [this](sensor_msgs::msg::Image::UniquePtr msg)
    {
      process<sensor_msgs::msg::Image::UniquePtr>(std::move(msg));
    };
    sub_ = node_.create_subscription<sensor_msgs::msg::Image>(input_topic, rclcpp::QoS(1), callback);
  }
} 

void ObjectDetectionSSD::initPublisher()
{
  std::string output_topic = node_.declare_parameter("output_topic").get<rclcpp::PARAMETER_STRING>();
  pub_ = node_.create_publisher<rdk_interfaces::msg::ObjectsInBoxes>(output_topic, rclcpp::QoS(1));
}

template <typename T>
void ObjectDetectionSSD::process(const T msg)
{
  objs_.header = msg->header;
  //debug
  //RCLCPP_INFO(node_.get_logger(), "timestamp: %d.%d, address: %p", msg->header.stamp.sec, msg->header.stamp.nanosec, reinterpret_cast<std::uintptr_t>(msg.get()));
  cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uchar *>(&msg->data[0]),
    msg->step);
  cv_width_ = msg->width;
  cv_height_ = msg->height;

  cv::Mat resized_image(cv_image);
  if (blob_width_ != cv_width_ || blob_height_ != cv_height_) {
    cv::resize(cv_image, resized_image, cv::Size(blob_width_, blob_height_));
  }

  if (is_first_frame_ == true) {
    is_first_frame_ = false;
  } else {
    async_infer_request_->Wait(IInferRequest::WaitMode::RESULT_READY);
  }
  Blob::Ptr image_input = async_infer_request_->GetBlob(input_name_);
  unsigned char* blob_data = static_cast<unsigned char*>(image_input->buffer());
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

void ObjectDetectionSSD::registerInferCompletionCallback()
{
  async_infer_request_ = exec_network_.CreateInferRequestPtr();
  Blob::Ptr image_input = async_infer_request_->GetBlob(input_name_);
  num_channels_ = image_input->getTensorDesc().getDims()[1];
  blob_width_ = image_input->getTensorDesc().getDims()[3];
  blob_height_ = image_input->getTensorDesc().getDims()[2];

  auto callback = [&] {

    objs_.objects_vector.clear();
    const float *detections = async_infer_request_->GetBlob(output_name_)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();

    for (int i = 0; i < max_proposal_count_; i++) {
      float image_id = detections[i*object_size_+0];
      int label = static_cast<int>(detections[i*object_size_+1]);
      float confidence = detections[i*object_size_+2];
      float xmin = detections[i*object_size_+3]*cv_width_;
      float ymin = detections[i*object_size_+4]*cv_height_;
      float xmax = detections[i*object_size_+5]*cv_width_;
      float ymax = detections[i*object_size_+6]*cv_height_;

      if (xmin > cv_width_ || xmax < 0 || ymin > cv_height_ || ymax < 0
        || (xmax-xmin)<=0 || (ymax-ymin)<=0) {
        continue;
      }
      xmin = (xmin < 0)? 0 : xmin;
      xmax = (xmax > cv_width_)? cv_width_ : xmax;
      ymin = (ymin < 0)? 0 : ymin;
      ymax = (ymax > cv_height_)? cv_height_ : ymax;

      if (confidence > 0.5) {
        rdk_interfaces::msg::ObjectInBox obj;
        if (!labels_.empty())
        {
          obj.object.object_name = labels_[label];
        }
        obj.object.probability = confidence;
        obj.roi.x_offset = xmin;
        obj.roi.y_offset = ymin;
        obj.roi.height = ymax - ymin;
        obj.roi.width = xmax- xmin;

        objs_.objects_vector.push_back(obj);
      }
    }
    pub_->publish(objs_);
  };
  async_infer_request_->SetCompletionCallback(callback);
}
}  // namespace openvino
