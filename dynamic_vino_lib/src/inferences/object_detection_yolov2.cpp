// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief a header file with declaration of ObjectDetection class and
 * ObjectDetectionResult class
 * @file object_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "dynamic_vino_lib/inferences/object_detection_yolov2.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"
/*
// ObjectDetectionResult
dynamic_vino_lib::ObjectDetectionYolov2Result::ObjectDetectionResult(const cv::Rect & location)
: Result(location)
{
}
*/

// ObjectDetection
dynamic_vino_lib::ObjectDetectionYolov2::ObjectDetectionYolov2(double show_output_thresh)
: show_output_thresh_(show_output_thresh),
  dynamic_vino_lib::ObjectDetection()
{
}

dynamic_vino_lib::ObjectDetectionYolov2::~ObjectDetectionYolov2() = default;

void dynamic_vino_lib::ObjectDetectionYolov2::loadNetwork(
  std::shared_ptr<Models::ObjectDetectionModel> network)
{
  valid_model_ = std::dynamic_pointer_cast<Models::ObjectDetectionYolov2Model>(network);
  if (valid_model_ == NULL)
  {
    std::cout << "ERROR" << std::endl;
  }

  max_proposal_count_ = network->getMaxProposalCount();
  object_size_ = network->getObjectSize();
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::ObjectDetectionYolov2::enqueue(
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  width_ = frame.cols;
  height_ = frame.rows;

  if (!matToBlob(frame, input_frame_loc, 1, 0, valid_model_->getInputName()))
  {
    return false;
  }

  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::ObjectDetectionYolov2::matToBlob(
        const cv::Mat& orig_image, const cv::Rect&, float scale_factor,
        int batch_index, const std::string& input_name)
{
  if (enqueued_frames_ == max_batch_size_)
    {
      slog::warn << "Number of " << getName() << "input more than maximum("
                 << max_batch_size_ << ") processed by inference" << slog::endl;
      return false;
    }
    InferenceEngine::Blob::Ptr input_blob =
        engine_->getRequest()->GetBlob(input_name);

  InferenceEngine::SizeVector blob_size = input_blob->getTensorDesc().getDims();
  const int width = blob_size[3];
  const int height = blob_size[2];
  const int channels = blob_size[1];
  float* blob_data = input_blob->buffer().as<float*>();

  int dx = 0;
  int dy = 0;
  int srcw = 0;
  int srch = 0;

  int IH = height;
  int IW = width;

  cv::Mat image = orig_image.clone();
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

  image.convertTo(image, CV_32F, 1.0/255.0, 0);
  srcw = image.size().width;
  srch = image.size().height;

  cv::Mat resizedImg (IH, IW, CV_32FC3);
  resizedImg = cv::Scalar(0.5, 0.5, 0.5);
  int imw = image.size().width;
  int imh = image.size().height;
  float resize_ratio = (float)IH / (float)std::max(imw, imh);
  cv::resize(image, image, cv::Size(imw*resize_ratio, imh*resize_ratio));

  int new_w = imw;
  int new_h = imh;
  if (((float)IW/imw) < ((float)IH/imh)) {
    new_w = IW;
    new_h = (imh * IW)/imw;
  } else {
    new_h = IH;
    new_w = (imw * IW)/imh;
  }
  dx = (IW-new_w)/2;
  dy = (IH-new_h)/2;

  imh = image.size().height;
  imw = image.size().width;

  for(int row = 0; row < imh; row ++)
  {
    for(int col = 0; col < imw; col ++)
    {
      for(int ch = 0; ch < 3; ch ++)
      {
        resizedImg.at<cv::Vec3f>(dy + row, dx + col)[ch] = image.at<cv::Vec3f>(row, col)[ch];
      }
    }
  }

  for (int c = 0; c < channels; c++)
  {
    for (int h = 0; h < height; h++)
    {
      for (int w = 0; w < width; w++)
      {
        blob_data[c * width * height + h*width + w] = resizedImg.at<cv::Vec3f>(h, w)[c];
      }
    }
  }

  ROI_.x = dx;
  ROI_.y = dy;
  ROI_.width = srcw;
  ROI_.height = srch;

  enqueued_frames_ += 1;
  return true;
}

bool dynamic_vino_lib::ObjectDetectionYolov2::submitRequest()
{
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::ObjectDetectionYolov2::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();

  if (!can_fetch) return false;
  bool found_result = false;
  results_.clear();

  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();

  std::string output = valid_model_->getOutputName();
  std::vector<std::string>& labels = valid_model_->getLabels();

  const float* detections = request->GetBlob(output)->buffer().as<InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type *>();
  InferenceEngine::CNNLayerPtr layer = valid_model_->getLayer();
  InferenceEngine::InputInfo::Ptr input_info = valid_model_->getInputInfo();

  int input_height = input_info->getTensorDesc().getDims()[2];
  int input_width = input_info->getTensorDesc().getDims()[3];

  // --------------------------- Validating output parameters -------------------------------------
  if (layer->type != "RegionYolo")
      throw std::runtime_error("Invalid output type: " + layer->type + ". RegionYolo expected");

  // --------------------------- Extracting layer parameters -------------------------------------
  const int num = layer->GetParamAsInt("num");
  const int coords = layer->GetParamAsInt("coords");
  const int classes = layer->GetParamAsInt("classes");

  const int out_blob_h = layer->input()->dims[0];

  std::vector<float> anchors = {
      0.572730, 0.677385,
      1.874460, 2.062530,
      3.338430, 5.474340,
      7.882820, 3.527780,
      9.770520, 9.168280
  };
  float threshold = 0.5;
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

      if (scale < threshold)
          continue;

      float x = (col + detections[box_index + 0 * side_square]) / side * input_width;
      float y = (row + detections[box_index + 1 * side_square]) / side * input_height;
      float height = std::exp(detections[box_index + 3 * side_square]) * anchors[2 * n + 1] / side * input_height;
      float width  = std::exp(detections[box_index + 2 * side_square]) * anchors[2 * n] / side * input_width;

      for (int j = 0; j < classes; ++j) {

        int class_index = getEntryIndex(side, coords, classes, n * side_square + i, coords + 1 + j);

        float prob = scale * detections[class_index];
        if (prob < threshold)
          continue;

        float x_min = x - width/2;
        float y_min = y - height/2;

        float x_min_resized = x_min / input_width * ROI_.width;
        float y_min_resized = y_min / input_height * ROI_.height;
        float width_resized = width / input_width * ROI_.width;
        float height_resized = height / input_height * ROI_.height;

        cv::Rect r(x_min_resized, y_min_resized, width_resized, height_resized);
        Result result(r);
        result.label_ = j;
        result.label_ = labels[j];

        result.confidence_ = prob;
        found_result = true;
        raw_results_.emplace_back(result);
      }
    }
  }

  std::sort(raw_results_.begin(), raw_results_.end());
  for (unsigned int i = 0; i < raw_results_.size(); ++i) {
    if (raw_results_[i].confidence_ == 0)
      continue;
      for (unsigned int j = i + 1; j < raw_results_.size(); ++j)
      if (IntersectionOverUnion(raw_results_[i], raw_results_[j]) >= 0.45)
        raw_results_[j].confidence_ = 0;
  }

  for (auto &raw_result : raw_results_) {
    if (raw_result.getConfidence() < 0.5)
       continue;

    results_.push_back(raw_result);
  }

  if (!found_result) {
    results_.clear();
  }
  raw_results_.clear();

  return true;
}

double dynamic_vino_lib::ObjectDetectionYolov2::IntersectionOverUnion(const Result &box_1, const Result &box_2)
{
  int xmax_1 = box_1.getLocation().x + box_1.getLocation().width;
  int xmin_1 = box_1.getLocation().x;
  int xmax_2 = box_2.getLocation().x + box_2.getLocation().width;
  int xmin_2 = box_2.getLocation().x;

  int ymax_1 = box_1.getLocation().y + box_1.getLocation().height;
  int ymin_1 = box_1.getLocation().y;
  int ymax_2 = box_2.getLocation().y + box_2.getLocation().height;
  int ymin_2 = box_2.getLocation().y;

  double width_of_overlap_area = fmin(xmax_1 , xmax_2) - fmax(xmin_1, xmin_2);
  double height_of_overlap_area = fmin(ymax_1, ymax_2) - fmax(ymin_1, ymin_2);
  double area_of_overlap;
  if (width_of_overlap_area < 0 || height_of_overlap_area < 0)
    area_of_overlap = 0;
  else
    area_of_overlap = width_of_overlap_area * height_of_overlap_area;

 double box_1_area = (ymax_1 - ymin_1)  * (xmax_1 - xmin_1);
 double box_2_area = (ymax_2 - ymin_2)  * (xmax_2 - xmin_2);
 double area_of_union = box_1_area + box_2_area - area_of_overlap;

 return area_of_overlap / area_of_union;
}

int dynamic_vino_lib::ObjectDetectionYolov2::getEntryIndex(int side, int lcoords, int lclasses, int location, int entry) {
  int n = location / (side * side);
  int loc = location % (side * side);
  return n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc;
}

const int dynamic_vino_lib::ObjectDetectionYolov2::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result * dynamic_vino_lib::ObjectDetectionYolov2::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::ObjectDetectionYolov2::getName() const
{
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::ObjectDetectionYolov2::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}
