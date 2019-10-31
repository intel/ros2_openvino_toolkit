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
  //debug
  //RCLCPP_INFO(node_.get_logger(), "timestamp: %d.%d, address: %p", msg->header.stamp.sec, msg->header.stamp.nanosec, reinterpret_cast<std::uintptr_t>(msg.get()));

  cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uchar *>(&msg->data[0]),
    msg->step);

  rdk_interfaces::msg::ObjectsInBoxes objs;
  objs.header = msg->header;
  process(cv_image, objs);
  pub_->publish(objs);
}

void ObjectDetectionSSD::process(cv::Mat & cv_image, rdk_interfaces::msg::ObjectsInBoxes & objs)
{
  InferRequest::Ptr async_infer_request = exec_network_.CreateInferRequestPtr();

  Blob::Ptr image_input = async_infer_request->GetBlob(input_name_);
  //auto t0 = node_.now();
  size_t num_channels = image_input->getTensorDesc().getDims()[1];
  size_t blob_width = image_input->getTensorDesc().getDims()[3];
  size_t blob_height = image_input->getTensorDesc().getDims()[2];
  //auto t1 = node_.now();
  //std::cout << "get width and height: " << (t1-t0).nanoseconds()*1e-6 <<std::endl;
  unsigned char* blob_data = static_cast<unsigned char*>(image_input->buffer());
  int cv_width = cv_image.cols;
  int cv_height = cv_image.rows;

  cv::Mat resized_image(cv_image);
  if (blob_width != cv_width || blob_height != cv_height) {
    cv::resize(cv_image, resized_image, cv::Size(blob_width, blob_height));
  }

  //auto t2 = node_.now();
  //std::cout << "resize image: " << (t2-t1).nanoseconds()*1e-6 <<std::endl;
  for (size_t c = 0; c < num_channels; c++) {
    for (size_t h = 0; h < blob_height; h++) {
      for (size_t w = 0; w < blob_width; w++) {
        blob_data[c * blob_width * blob_height + h * blob_width + w] =
          resized_image.at<cv::Vec3b>(h, w)[c];
      }
    }
  }
  //auto t3 = node_.now();
  //std::cout << "fill the blob: " << (t3-t2).nanoseconds()*1e-6 <<std::endl;

  rdk_interfaces::msg::ObjectInBox obj;

  async_infer_request->StartAsync();
  async_infer_request->Wait(IInferRequest::WaitMode::RESULT_READY);

  const float *detections = async_infer_request->GetBlob(output_name_)->buffer().as<PrecisionTrait<Precision::FP32>::value_type*>();
  
  for (int i = 0; i < max_proposal_count_; i++) {
    float image_id = detections[i*object_size_+0];
    int label = static_cast<int>(detections[i*object_size_+1]);
    float confidence = detections[i*object_size_+2];
    float xmin = detections[i*object_size_+3]*cv_width;
    float ymin = detections[i*object_size_+4]*cv_height;
    float xmax = detections[i*object_size_+5]*cv_width;
    float ymax = detections[i*object_size_+6]*cv_height;

    if (xmin > cv_width || xmax < 0 || ymin > cv_height || ymax < 0
      || (xmax-xmin)<=0 || (ymax-ymin)<=0) {
      continue;
    }
    xmin = (xmin < 0)? 0 : xmin;
    xmax = (xmax > cv_width)? cv_width : xmax;
    ymin = (ymin < 0)? 0 : ymin;
    ymax = (ymax > cv_height)? cv_height : ymax;

    // show image
    if (confidence > 0.2) {
      #if 0
      std::ostringstream conf;
      conf << ":" << std::fixed << std::setprecision(3) << confidence;

      cv::putText(cv_image,
       (label < getLabels().size() ? getLabels()[label] : std::string("label #") + std::to_string(label))
       + conf.str(),
      cv::Point2f(xmin, ymin - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
       cv::Scalar(0, 0, 255));

      cv::rectangle(cv_image, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), cv::Scalar(0, 0, 255));
      cv::imshow("Detection results", cv_image);
      cv::waitKey(1);
      #endif
      if (!labels_.empty())
      {
        obj.object.object_name = labels_[label];
      }
      obj.object.probability = confidence;
      obj.roi.x_offset = xmin;
      obj.roi.y_offset = ymin;
      obj.roi.height = ymax - ymin;
      obj.roi.width = xmax- xmin;

      objs.objects_vector.push_back(obj);
    }
  }
}
}  // namespace openvino
