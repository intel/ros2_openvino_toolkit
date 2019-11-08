#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "object_msgs/msg/object_in_box.hpp"
#include "object_msgs/msg/object_in_mask.hpp"
#include "object_msgs/msg/object.hpp"
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
  pub_ = node_.create_publisher<object_msgs::msg::ObjectsInMasks>(output_topic, 16);
}

template <typename T>
void ObjectSegmentation::process(const T msg)
{
  objs_.header = msg->header;
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

  Blob::Ptr image_tensor_input = async_infer_request_->GetBlob(input_tensor_name_);
  auto blob_data = image_tensor_input->buffer().as<PrecisionTrait<Precision::U8>::value_type *>();
   
  for (size_t c = 0; c < num_channels_; c++) {
    for (size_t h = 0; h < blob_height_; h++) {
      for (size_t w = 0; w < blob_width_; w++) {
        blob_data[c * blob_width_ * blob_height_ + h * blob_width_ + w] =
          resized_image.at<cv::Vec3b>(h, w)[c];
      }
    }
  }

  Blob::Ptr image_info_input = async_infer_request_->GetBlob(input_info_name_);
  auto blob_info = image_info_input->buffer().as<PrecisionTrait<Precision::FP32>::value_type *>();
  blob_info[0] = static_cast<float>(blob_height_);  // height
  blob_info[1] = static_cast<float>(blob_width_);  // width
  blob_info[2] = 1;

  async_infer_request_->StartAsync();
}

void ObjectSegmentation::registerInferCompletionCallback()
{
  async_infer_request_ = exec_network_.CreateInferRequestPtr();

  Blob::Ptr image_tensor_input = async_infer_request_->GetBlob(input_tensor_name_);
  num_channels_ = image_tensor_input->getTensorDesc().getDims()[1];
  blob_height_ = image_tensor_input->getTensorDesc().getDims()[2];
  blob_width_ = image_tensor_input->getTensorDesc().getDims()[3];

  auto callback = [&] {
    objs_.objects_vector.clear();
    object_msgs::msg::ObjectInMask obj;

    const auto do_blob = async_infer_request_->GetBlob(detection_output_name_);
    const auto do_data = do_blob->buffer().as<float*>();

    const auto masks_blob = async_infer_request_->GetBlob(mask_output_name_);
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

      /*
      float x1 = std::min(std::max(0.0f, box_info[3] * cv_image.size().width), static_cast<float>(cv_image.size().width));
      float y1 = std::min(std::max(0.0f, box_info[4] * cv_image.size().height), static_cast<float>(cv_image.size().height));
      float x2 = std::min(std::max(0.0f, box_info[5] * cv_image.size().width), static_cast<float>(cv_image.size().width));
      float y2 = std::min(std::max(0.0f, box_info[6] * cv_image.size().height), static_cast<float>(cv_image.size().height));
      int box_width = std::min(static_cast<int>(std::max(0.0f, x2 - x1)), cv_image.size().width);
      int box_height = std::min(static_cast<int>(std::max(0.0f, y2 - y1)), cv_image.size().height);
      */
      float x1 = std::min(std::max(0.0f, box_info[3] * cv_width_), static_cast<float>(cv_width_));
      float y1 = std::min(std::max(0.0f, box_info[4] * cv_height_), static_cast<float>(cv_height_));
      float x2 = std::min(std::max(0.0f, box_info[5] * cv_width_), static_cast<float>(cv_width_));
      float y2 = std::min(std::max(0.0f, box_info[6] * cv_height_), static_cast<float>(cv_height_));
      int box_width = std::min(static_cast<int>(std::max(0.0f, x2 - x1)), static_cast<int>(cv_width_));
      int box_height = std::min(static_cast<int>(std::max(0.0f, y2 - y1)), static_cast<int>(cv_height_));

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
        objs_.objects_vector.push_back(obj);
      }
    }
    pub_->publish(objs_);
  };
  async_infer_request_->SetCompletionCallback(callback);
}
}  // namespace openvino
