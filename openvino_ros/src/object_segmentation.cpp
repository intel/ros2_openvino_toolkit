#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "rdk_interfaces/msg/object_in_box.hpp"
#include "rdk_interfaces/msg/object_in_mask.hpp"
#include "rdk_interfaces/msg/object.hpp"
#include "openvino/object_segmentation.hpp"

using namespace InferenceEngine;

namespace openvino
{
ObjectSegmentation::ObjectSegmentation(rclcpp::Node & node)
: OpenVINOBase(node)
{
  init();
}

void ObjectSegmentation::prepareInputBlobs()
{
  InputsDataMap input_info(network_.getInputsInfo());

  for (const auto & input_info_item : input_info) {
    if (input_info_item.second->getDims().size() == 4) {
      input_tensor_name_ = input_info_item.first;
      input_info_item.second->setPrecision(Precision::U8);
    }
    else if (input_info_item.second->getDims().size() ==2) {
      input_info_name_ = input_info_item.first;
      input_info_item.second->setPrecision(Precision::FP32);
    }
  }
}

void ObjectSegmentation::prepareOutputBlobs()
{
  network_.addOutput(std::string("detection_output"), 0);
  OutputsDataMap output_info(network_.getOutputsInfo());
  for (auto & item : output_info) {
    item.second->setPrecision(Precision::FP32);
  }

  auto output_ptr = output_info.begin();
  detection_output_name_ = output_ptr->first;
  mask_output_name_ = (++output_ptr)->first;
}

void ObjectSegmentation::initSubscriber()
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

void ObjectSegmentation::initPublisher()
{
  std::string output_topic = node_.declare_parameter("output_topic").get<rclcpp::PARAMETER_STRING>();
  pub_ = node_.create_publisher<rdk_interfaces::msg::ObjectsInMasks>(output_topic, 16);
}

template <typename T>
void ObjectSegmentation::process(const T msg)
{
  //debug
  //RCLCPP_INFO(node_.get_logger(), "timestamp: %d.%d, address: %p", msg->header.stamp.sec, msg->header.stamp.nanosec, reinterpret_cast<std::uintptr_t>(msg.get()));
  
  cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uchar *>(&msg->data[0]),
    msg->step);

  rdk_interfaces::msg::ObjectsInMasks objs;
  objs.header = msg->header;
  process(cv_image, objs);
  pub_->publish(objs);
}

void ObjectSegmentation::process(cv::Mat & cv_image, rdk_interfaces::msg::ObjectsInMasks & objs)
{
  InferRequest::Ptr async_infer_request = exec_network_.CreateInferRequestPtr();
  
  Blob::Ptr image_tensor_input = async_infer_request->GetBlob(input_tensor_name_);
  auto blob_data = image_tensor_input->buffer().as<PrecisionTrait<Precision::U8>::value_type *>();
  size_t batch_size = image_tensor_input->getTensorDesc().getDims()[0];
  size_t num_channels = image_tensor_input->getTensorDesc().getDims()[1];
  size_t blob_height = image_tensor_input->getTensorDesc().getDims()[2];
  size_t blob_width = image_tensor_input->getTensorDesc().getDims()[3];

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

  Blob::Ptr image_info_input = async_infer_request->GetBlob(input_info_name_);
  auto blob_info = image_info_input->buffer().as<PrecisionTrait<Precision::FP32>::value_type *>();
  blob_info[0] = static_cast<float>(blob_height);  // height
  blob_info[1] = static_cast<float>(blob_width);  // width
  blob_info[2] = 1;

  rdk_interfaces::msg::ObjectInMask obj;

  async_infer_request->StartAsync();
  async_infer_request->Wait(IInferRequest::WaitMode::RESULT_READY);
  
  const auto do_blob = async_infer_request->GetBlob(detection_output_name_);
  const auto do_data = do_blob->buffer().as<float*>();

  const auto masks_blob = async_infer_request->GetBlob(mask_output_name_);
  const auto masks_data = masks_blob->buffer().as<float*>();

  const float probability_threshold = 0.2f;

  size_t box_description_size = do_blob->dims().at(0);
  size_t boxes = masks_blob->dims().at(3);

  size_t C = masks_blob->dims().at(2);
  size_t H = masks_blob->dims().at(1);
  size_t W = masks_blob->dims().at(0);

  size_t box_stride = W * H * C;

  // some colours
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
  std::map<size_t, size_t> class_color;

  /** Iterating over all boxes **/
  for (size_t box = 0; box < boxes; ++box) {
    float* box_info = do_data + box * box_description_size;
    float prob = box_info[2];

    float x1 = std::min(std::max(0.0f, box_info[3] * cv_image.size().width), static_cast<float>(cv_image.size().width));
    float y1 = std::min(std::max(0.0f, box_info[4] * cv_image.size().height), static_cast<float>(cv_image.size().height));
    float x2 = std::min(std::max(0.0f, box_info[5] * cv_image.size().width), static_cast<float>(cv_image.size().width));
    float y2 = std::min(std::max(0.0f, box_info[6] * cv_image.size().height), static_cast<float>(cv_image.size().height));
    int box_width = std::min(static_cast<int>(std::max(0.0f, x2 - x1)), cv_image.size().width);
    int box_height = std::min(static_cast<int>(std::max(0.0f, y2 - y1)), cv_image.size().height);
    auto class_id = static_cast<size_t>(box_info[1] + 1e-6f);

    if (prob > probability_threshold) {

      if (class_color.find(class_id) == class_color.end())
        class_color[class_id] = class_color.size();

      auto& color = colors[class_color[class_id]];
      float* mask_arr = masks_data + box_stride * box + H * W * (class_id - 1);
      cv::Mat mask_mat(H, W, CV_32FC1, mask_arr);
      cv::Rect roi = cv::Rect(static_cast<int>(x1), static_cast<int>(y1), box_width, box_height);
      cv::Mat resized_mask_mat(box_height, box_width, CV_32FC1);
      cv::resize(mask_mat, resized_mask_mat, cv::Size(box_width, box_height));
      if (!labels_.empty())
      {
        obj.object_name = labels_[class_id];
      }
      obj.probability = prob;
      obj.roi.x_offset = roi.x;
      obj.roi.y_offset = roi.y;
      obj.roi.width = roi.width;
      obj.roi.height = roi.height;

      for (int h = 0; h < resized_mask_mat.size().height; ++h) {
        for (int w = 0; w < resized_mask_mat.size().width; ++w) {
          obj.mask_array.push_back(resized_mask_mat.at<float>(h, w));
        }
      }

      objs.objects_vector.push_back(obj);
    }
  }
}
}  // namespace openvino
