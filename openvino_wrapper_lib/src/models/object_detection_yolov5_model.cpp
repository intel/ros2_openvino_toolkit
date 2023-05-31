// Copyright (c) 2022-2023 Intel Corporation
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
 * @brief a header file with declaration of ObjectDetectionYolov5Model class
 * @file object_detection_yolov5_model.cpp
 */
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/utils/common.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/inferences/object_detection.hpp"
#include "openvino_wrapper_lib/models/object_detection_yolov5_model.hpp"

using namespace cv;
using namespace dnn;

// Validated Object Detection Network
Models::ObjectDetectionYolov5Model::ObjectDetectionYolov5Model(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: ObjectDetectionModel(label_loc, model_loc, max_batch_size)
{
  setKeepInputShapeRatio(true);
}

bool Models::ObjectDetectionYolov5Model::updateLayerProperty(
  std::shared_ptr<ov::Model>& model)
{
  Models::BaseModel::updateLayerProperty(model); 

  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);

  // preprocess image inputs
  ov::preprocess::InputInfo& input_info = ppp.input(getInputInfo("input0"));
  ov::Layout tensor_layout = ov::Layout("NHWC");

  if( model->input(0).get_partial_shape().is_dynamic()){
    auto expected_size = getExpectedFrameSize();
    slog::info << "Model's input has dynamic shape, set to expected size: "
               << expected_size << slog::endl;
    input_info.tensor().set_shape({1, expected_size.height, expected_size.width, 3});
  }

  input_info.tensor().
  set_element_type(ov::element::u8).
  set_layout(tensor_layout).
  set_color_format(ov::preprocess::ColorFormat::BGR);

  input_info.preprocess().
    convert_element_type(ov::element::f32).
    convert_color(ov::preprocess::ColorFormat::RGB).scale({255., 255., 255.});
  ppp.input().model().set_layout("NCHW");

  ppp.output().tensor().set_element_type(ov::element::f32);

  model = ppp.build();

  ov::Shape input_shape = model->input(getInputInfo("input0")).get_shape();
  slog::debug<<"image_tensor shape is:"<< input_shape.size() <<slog::endl;
  OPENVINO_ASSERT (input_shape.size()== 4);
  setInputHeight(input_shape[1]);
  setInputWidth(input_shape[2]);

  auto output_info_map = model->outputs();
  ov::Shape output_dims = output_info_map[0].get_shape();
  if (output_dims[1] < output_dims[2]){
    slog::info << "Object-Size bigger than Proposal-Count, Outputs need Transform!" << slog::endl;
    setTranspose(true);
    setMaxProposalCount(static_cast<int>(output_dims[2]));
    setObjectSize(static_cast<int>(output_dims[1]));
  } else {
    setTranspose(false);
    setMaxProposalCount(static_cast<int>(output_dims[1]));
    setObjectSize(static_cast<int>(output_dims[2]));
  }
  printAttribute();
  slog::info << "This model is Yolo-like, Layer Property updated!" << slog::endl;
  return true;
}

const std::string Models::ObjectDetectionYolov5Model::getModelCategory() const
{
  return "Object Detection Yolo v5";
}

bool Models::ObjectDetectionYolov5Model::enqueue(
  const std::shared_ptr<Engines::Engine> & engine,
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  setFrameSize(frame.cols, frame.rows);

  if (!matToBlob(frame, input_frame_loc, 1, 0, engine)) {
    return false;
  }
  return true;
}

bool Models::ObjectDetectionYolov5Model::fetchResults(
  const std::shared_ptr<Engines::Engine> & engine,
  std::vector<openvino_wrapper_lib::ObjectDetectionResult> & results,
  const float & confidence_thresh,
  const bool & enable_roi_constraint)
{
  const float NMS_THRESHOLD = 0.45;   //  remove overlapping bounding boxes

  ov::InferRequest request = engine->getRequest();
  std::string output = getOutputName();
  const ov::Tensor &output_tensor = request.get_output_tensor();
  ov::Shape output_shape = output_tensor.get_shape();
  auto *detections = output_tensor.data<float>();
  int rows = output_shape.at(1);
  int dimentions = output_shape.at(2);
  Mat output_buffer(output_shape[1], output_shape[2], CV_32F, detections);
  //Check if transpose is needed
  if (output_shape.at(2) > output_shape.at(1) &&
      output_shape.at(2) > 300){ // 300 is just a random number(bigger than the number of classes)
    transpose(output_buffer, output_buffer); //[8400,84] for yolov8
    detections = (float*)output_buffer.data;
    rows = output_shape.at(2);
    dimentions = output_shape.at(1);
  }
  //slog::debug << "AFTER calibration: rows->" << rows << ", dimentions->" << dimentions << slog::endl; 

  std::vector<cv::Rect> boxes;
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<std::string> & labels = getLabels();

  for (size_t i = 0; i < rows; i++) {
    float *detection = &detections[i * dimentions];
    if (hasConfidenceOutput()) {
      float confidence = detection[4];
      if (confidence < confidence_thresh)
        continue;
    }

    const int classes_scores_start_pos = hasConfidenceOutput()? 5 : 4;
    float *classes_scores = &detection[classes_scores_start_pos];
    int col = static_cast<int>(dimentions - classes_scores_start_pos);

    cv::Mat scores(1, col, CV_32FC1, classes_scores);
    cv::Point class_id;
    double max_class_score;
    cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id);

    if (max_class_score > confidence_thresh) {
        confidences.emplace_back(max_class_score);
        class_ids.emplace_back(class_id.x);

        float x = detection[0];
        float y = detection[1];
        float w = detection[2];
        float h = detection[3];
        auto x_min = x - (w / 2);
        auto y_min = y - (h / 2);

        boxes.emplace_back(x_min, y_min, w, h);
    }
  }

  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, confidence_thresh, NMS_THRESHOLD, nms_result);
  for (int idx: nms_result) {
      double rx = getFrameResizeRatioWidth();
      double ry = getFrameResizeRatioHeight();
      int vx = int(rx * boxes[idx].x);
      double vy = int(ry * boxes[idx].y);
      double vw = int(rx * boxes[idx].width);
      double vh = int(ry * boxes[idx].height);
      cv::Rect rec(vx, vy, vw, vh);
      Result result(rec);
      result.setConfidence(confidences[idx]);
      std::string label = class_ids[idx] < labels.size() ?
        labels[class_ids[idx]] : std::string("label #") + std::to_string(class_ids[idx]);
      result.setLabel(label);
      results.push_back(result);
  }

  return true;
}
