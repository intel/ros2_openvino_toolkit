#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include "openvino/object_detection_yolov2.hpp"

using namespace InferenceEngine;

namespace openvino
{
ObjectDetectionYOLOV2::ObjectDetectionYOLOV2(rclcpp::Node & node)
: OpenVINOBase(node)
{
  init();
}

void ObjectDetectionYOLOV2::prepareInputBlobs()
{
  InputsDataMap input_info(network_.getInputsInfo());
  InputInfo::Ptr & input = input_info.begin()->second;
  if (input == nullptr) {
    RCLCPP_ERROR(node_.get_logger(), "Input info of Yolov2 model should not be nullptr");
    rclcpp::shutdown();
  }
  input_name_ = input_info.begin()->first;
  input->setPrecision(Precision::FP32);
  input->getInputData()->setLayout(Layout::NCHW);
}

void ObjectDetectionYOLOV2::prepareOutputBlobs()
{
  OutputsDataMap output_info(network_.getOutputsInfo());
  DataPtr & output = output_info.begin()->second;
  output_name_ = output_info.begin()->first;

  output->setPrecision(Precision::FP32);
}

void ObjectDetectionYOLOV2::initSubscriber()
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

void ObjectDetectionYOLOV2::initPublisher()
{
  std::string output_topic = node_.declare_parameter("output_topic").get<rclcpp::PARAMETER_STRING>();
  pub_ = node_.create_publisher<object_msgs::msg::ObjectsInBoxes>(output_topic, 16);
}

template <typename T>
void ObjectDetectionYOLOV2::process(const T msg)
{
  objs_.header = msg->header;

  cv::Mat cv_image(msg->height, msg->width, CV_8UC3, const_cast<uchar *>(&msg->data[0]), 
    msg->step);

  if (is_first_frame_ == true) {
    is_first_frame_ = false;
  } else {
    async_infer_request_->Wait(IInferRequest::WaitMode::RESULT_READY);
  }

  Blob::Ptr image_input = async_infer_request_->GetBlob(input_name_);

  size_t num_channels = image_input->getTensorDesc().getDims()[1];
  size_t blob_width = image_input->getTensorDesc().getDims()[3];
  size_t blob_height = image_input->getTensorDesc().getDims()[2];

  float * blob_data = image_input->buffer().as<float *>();

  int dx = 0;
  int dy = 0;

  int IH = blob_height;
  int IW = blob_width;

  cv::Mat image = cv_image.clone();
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  image.convertTo(image, CV_32F, 1.0 / 255.0, 0);
  srcw_ = image.size().width;
  srch_ = image.size().height;

  cv::Mat resizedImg(IH, IW, CV_32FC3);
  resizedImg = cv::Scalar(0.5, 0.5, 0.5);
  imw_ = image.size().width;
  imh_ = image.size().height;

  float resize_ratio = static_cast<float>(IH) / static_cast<float>(std::max(imw_, imh_));
  cv::resize(image, image, cv::Size(imw_ * resize_ratio, imh_ * resize_ratio));

  int new_w = imw_;
  int new_h = imh_;
  if ((static_cast<float>(IW) / imw_) < (static_cast<float>(IH) / imh_)) {
    new_w = IW;
    new_h = (imh_ * IW) / imw_;
  } else {
    new_h = IH;
    new_w = (imw_ * IW) / imh_;
  }
  dx = (IW - new_w) / 2;
  dy = (IH - new_h) / 2;

  imh_ = image.size().height;
  imw_ = image.size().width;

  for (int row = 0; row < imh_; row++) {
    for (int col = 0; col < imw_; col++) {
      for (int ch = 0; ch < 3; ch++) {
        resizedImg.at<cv::Vec3f>(dy + row, dx + col)[ch] = image.at<cv::Vec3f>(row, col)[ch];
      }
    }
  }

  for (int c = 0; c < num_channels; c++) {
    for (int h = 0; h < blob_height; h++) {
      for (int w = 0; w < blob_width; w++) {
        blob_data[c * blob_width * blob_height + h * blob_width + w] = resizedImg.at<cv::Vec3f>(h, w)[c];
      }
    }
  }

  async_infer_request_->StartAsync();
}

void ObjectDetectionYOLOV2::registerInferCompletionCallback()
{
  async_infer_request_ = exec_network_.CreateInferRequestPtr();
  Blob::Ptr image_input = async_infer_request_->GetBlob(input_name_);
  size_t num_channels = image_input->getTensorDesc().getDims()[1];
  size_t blob_width = image_input->getTensorDesc().getDims()[3];
  size_t blob_height = image_input->getTensorDesc().getDims()[2];

  auto callback = [&] {
  
    objs_.objects_vector.clear();
    const float * detections = async_infer_request_->GetBlob(output_name_)->buffer().as<PrecisionTrait<Precision::FP32>::value_type *>();

    const int num = network_.getLayerByName(output_name_.c_str())->GetParamAsInt("num");
    const int coords = network_.getLayerByName(output_name_.c_str())->GetParamAsInt("coords");
    const int classes = network_.getLayerByName(output_name_.c_str())->GetParamAsInt("classes");

    const int out_blob_h = network_.getLayerByName(output_name_.c_str())->input()->dims[0]; 

    std::vector<float> anchors = {
      0.572730, 0.677385,
      1.874460, 2.062530,
      3.338430, 5.474340,
      7.882820, 3.527780,
      9.770520, 9.168280
    };
    auto side = out_blob_h;

    auto side_square = side * side;

    // --------------------------- Parsing YOLO Region output -------------------------------------
    for (int i = 0; i < side_square; ++i) {
      int row = i / side;
      int col = i % side;

      for (int n = 0; n < num; ++n) {
        int obj_index = getEntryIndex(side, coords, classes, n * side * side + i, coords);
        int box_index = getEntryIndex(side, coords, classes, n * side * side + i, 0);

        float scale = detections[obj_index];

        if (scale < 0.7) {
          continue;
        }

        float x = (col + detections[box_index + 0 * side_square]) / side * blob_width;
        float y = (row + detections[box_index + 1 * side_square]) / side * blob_height;
        float height = std::exp(detections[box_index + 3 * side_square]) * anchors[2 * n + 1] /
          side * blob_height;
        float width = std::exp(detections[box_index + 2 * side_square]) * anchors[2 * n] / side *
          blob_width;

        for (int j = 0; j < classes; ++j) {
          int class_index =
            getEntryIndex(side, coords, classes, n * side_square + i, coords + 1 + j);

          float prob = scale * detections[class_index];
          if (prob < 0.7) {
            continue;
          }

          float x_min = x - width / 2;
          float x_max = x + width / 2;
          float y_min = y - height / 2;
          float y_max = y + height / 2;

          float x_min_resized = x_min / imw_ * srcw_;
          float y_min_resized = y_min / imh_ * srch_;
          float x_max_resized = x_max / imw_ * srcw_;
          float y_max_resized = y_max / imh_ * srch_;

          object_msgs::msg::ObjectInBox obj;
          std::string label = j <
            labels_.size() ? labels_[j] : std::string("label #") + std::to_string(j);

	  // image window display
	  /*
	  std::ostringstream conf;
          conf << ":" << std::fixed << std::setprecision(3) << prob;

          cv::putText(cv_image, label + conf.str(),
            cv::Point2f(x_min_resized, y_min_resized - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
            cv::Scalar(0, 0, 255));

          cv::rectangle(cv_image, cv::Point2f(x_min_resized, y_min_resized), cv::Point2f(x_max_resized, y_max_resized), cv::Scalar(0, 0, 255));
          cv::imshow("Detection results", cv_image);
          cv::waitKey(1);
	  */

          obj.object.object_name = label;
          obj.object.probability = prob;
          obj.roi.x_offset = x_min;
          obj.roi.y_offset = y_min;
          obj.roi.height = y_max - y_min;
          obj.roi.width = x_max- x_min;

          objs_.objects_vector.push_back(obj);
        }
      }
    }

    std::sort(objs_.objects_vector.begin(), objs_.objects_vector.end(), sortByProbility);
    for (unsigned int i = 0; i < objs_.objects_vector.size(); ++i) {
      if (objs_.objects_vector[i].object.probability == 0) {
        continue;
      }
      for (unsigned int j = i + 1; j < objs_.objects_vector.size(); ++j) {
        cv::Rect location_i(objs_.objects_vector[i].roi.x_offset,
          objs_.objects_vector[i].roi.y_offset, objs_.objects_vector[i].roi.width,
	  objs_.objects_vector[i].roi.height);

        cv::Rect location_j(objs_.objects_vector[j].roi.x_offset,
          objs_.objects_vector[j].roi.y_offset, objs_.objects_vector[j].roi.width,
	  objs_.objects_vector[j].roi.height);

        auto iou = ObjectDetectionYOLOV2::intersectionOverUnion(
          location_i, location_j);
        if (iou >= 0.45) {
          objs_.objects_vector[j].object.probability = 0;
        }
      }
    }
    pub_->publish(objs_);
  };
  async_infer_request_->SetCompletionCallback(callback);
}

double ObjectDetectionYOLOV2::intersectionOverUnion(const cv::Rect & box_1,
  const cv::Rect & box_2)
{
  int xmax_1 = box_1.x + box_1.width;
  int xmin_1 = box_1.x;
  int xmax_2 = box_2.x + box_2.width;
  int xmin_2 = box_2.x;

  int ymax_1 = box_1.y + box_1.height;
  int ymin_1 = box_1.y;
  int ymax_2 = box_2.y + box_2.height;
  int ymin_2 = box_2.y;

  double width_of_overlap_area = fmin(xmax_1, xmax_2) - fmax(xmin_1, xmin_2);
  double height_of_overlap_area = fmin(ymax_1, ymax_2) - fmax(ymin_1, ymin_2);
  double area_of_overlap;
  if (width_of_overlap_area < 0 || height_of_overlap_area < 0) {
    area_of_overlap = 0;
  } else {
    area_of_overlap = width_of_overlap_area * height_of_overlap_area;
  }

  double box_1_area = (ymax_1 - ymin_1) * (xmax_1 - xmin_1);
  double box_2_area = (ymax_2 - ymin_2) * (xmax_2 - xmin_2);
  double area_of_union = box_1_area + box_2_area - area_of_overlap;

  return area_of_overlap / area_of_union;
}

int ObjectDetectionYOLOV2::getEntryIndex(int side, int lcoords, int lclasses, int location,
  int entry)
{
  int n = location / (side * side);
  int loc = location % (side * side);
  return n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc;
}

}  // namespace openvino
