// Copyright (c) 2022 Intel Corporation
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
 * @brief a header file with declaration of ObjectDetectionModel class
 * @file object_detection_yolov5_model.cpp
 */
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/inferences/object_detection.hpp"
#include "openvino_wrapper_lib/models/object_detection_yolov5_model.hpp"


// Validated Object Detection Network
Models::ObjectDetectionYolov5Model::ObjectDetectionYolov5Model(
  const std::string & label_loc, const std::string & model_loc, int max_batch_size)
: ObjectDetectionModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::ObjectDetectionYolov5Model::updateLayerProperty(
  std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;
  auto input_info_map = model->inputs();
  if (input_info_map.size() != 1) {
    slog::warn << "This model seems not Yolo-like, which has only one input, but we got "
      << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    return false;
  }
  // set input property
  ov::Shape input_dims = input_info_map[0].get_shape();
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  const ov::Layout input_tensor_layout{"NHWC"};
  setInputHeight(input_dims[2]);
  setInputWidth(input_dims[3]);
  input_info.tensor().
    set_element_type(ov::element::u8).
    set_layout(input_tensor_layout).
    set_color_format(ov::preprocess::ColorFormat::BGR);
  input_info.preprocess().
    convert_element_type(ov::element::f32).
    convert_color(ov::preprocess::ColorFormat::RGB).scale({255., 255., 255.});
  ppp.input().model().set_layout("NCHW");
  addInputInfo("input", input_tensor_name_);

  // set output property
  auto output_info_map = model->outputs();
  if (output_info_map.size() != 1) {
    slog::warn << "This model seems not Yolo-like! We got "
      << std::to_string(output_info_map.size()) << "outputs, but Yolov5 has only one."
      << slog::endl;
    return false;
  }
  output_tensor_name_ = model->output().get_any_name();
  ov::preprocess::OutputInfo& output_info = ppp.output();
  addOutputInfo("output", output_tensor_name_);
  output_info.tensor().set_element_type(ov::element::f32);
  slog::info << "Checking Object Detection output ... Name=" << output_tensor_name_
    << slog::endl;

  model = ppp.build();

  ov::Shape output_dims = output_info_map[0].get_shape();
  setMaxProposalCount(static_cast<int>(output_dims[1]));

  auto object_size = static_cast<int>(output_dims[2]);
  setObjectSize(object_size);

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


bool Models::ObjectDetectionYolov5Model::matToBlob(
  const cv::Mat & orig_image, const cv::Rect &, float scale_factor,
  int batch_index, const std::shared_ptr<Engines::Engine> & engine)
{
  pre_process_ov(orig_image);
  input_image = orig_image;
  dataToBlob(resize_img.resized_image, scale_factor, batch_index, engine, false);

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
  std::vector<cv::Rect> boxes;
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<std::string> & labels = getLabels();

  for (size_t i = 0; i < output_shape.at(1); i++) {
    float *detection = &detections[i * output_shape.at(2)];
    float confidence = detection[4];
    if (confidence < confidence_thresh)
      continue;

    float *classes_scores = &detection[5];
    int col = static_cast<int>(output_shape.at(2) - 5);

    cv::Mat scores(1, col, CV_32FC1, classes_scores);
    cv::Point class_id;
    double max_class_score;
    cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id);

    if (max_class_score > confidence_thresh) {
        confidences.emplace_back(confidence);
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
      double rx = (double) input_image.cols / (resize_img.resized_image.cols - resize_img.dw);
      double ry = (double) input_image.rows / (resize_img.resized_image.rows - resize_img.dh);
      double vx = rx * boxes[idx].x;
      double vy = ry * boxes[idx].y;
      double vw = rx * boxes[idx].width;
      double vh = ry * boxes[idx].height;
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

bool Models::ObjectDetectionYolov5Model::pre_process_ov(const cv::Mat &input_image) {
    const float INPUT_WIDTH = 640.f;
    const float INPUT_HEIGHT = 640.f;
    auto width = (float) input_image.cols;
    auto height = (float) input_image.rows;
    auto r = float(INPUT_WIDTH / std::max(width, height));
    int new_unpadW = int(round(width * r));
    int new_unpadH = int(round(height * r));

    resize_img.dw = (int) INPUT_WIDTH - new_unpadW;
    resize_img.dh = (int) INPUT_HEIGHT - new_unpadH;
    resize_img.resized_image = resizeImage(input_image, new_unpadW, new_unpadH, 0, 0, cv::INTER_AREA,
                                         0, resize_img.dh, 0, resize_img.dw, cv::Scalar(100, 100, 100));
    return true;
}
