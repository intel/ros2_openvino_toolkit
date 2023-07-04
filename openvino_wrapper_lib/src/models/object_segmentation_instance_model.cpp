// Copyright (c) 2023 Intel Corporation
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
 * @brief a header file with declaration of ObjectSegmentationInstanceModel class
 * By default yolov8 segementation models are supported.
 */
#include <string>
#include <vector>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/inferences/object_segmentation_instance.hpp"
#include "openvino_wrapper_lib/utils/common.hpp"
#include "openvino_wrapper_lib/models/object_segmentation_instance_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"

// Validated Object Segmentation Network
Models::ObjectSegmentationInstanceModel::ObjectSegmentationInstanceModel(
    const std::string & label_loc, 
    const std::string & model_loc,
    int max_batch_size)
    : BaseModel(label_loc, model_loc, max_batch_size)
{
  setHasConfidenceOutput(false);
  setKeepInputShapeRatio(true);
  setCountOfInputs(1);
  setCountOfOutputs(2);
  setExpectedFrameSize({640, 640});
}

bool Models::ObjectSegmentationInstanceModel::enqueue(
    const std::shared_ptr<Engines::Engine> &engine,
    const cv::Mat &frame,
    const cv::Rect &input_frame_loc)
{
  if (engine == nullptr)
  {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  setFrameSize(frame.cols, frame.rows);

  for (const auto &inputInfoItem : inputs_info_)
  {
    auto dims = inputInfoItem.get_partial_shape();
    slog::debug << "input tensor shape is:"<< dims.size() <<slog::endl;
    
    if (dims.size()==4)
    {
      matToBlob(frame, input_frame_loc, 1.0, 0, engine);
    }

    // Fill second input tensor with image info
    if (dims.size() == 2)
    {
      ov::Tensor in_tensor = engine->getRequest().get_tensor(inputInfoItem);
      auto data = in_tensor.data<float>();
      data[0] = static_cast<float>(frame.rows); // height
      data[1] = static_cast<float>(frame.cols);  // width
      data[2] = 1;
    }
  }
 
  return true;
}

const std::string Models::ObjectSegmentationInstanceModel::getModelCategory() const
{
  return "Object Segmentation - Yolo-Like";
}

bool Models::ObjectSegmentationInstanceModel::updateLayerProperty(
    std::shared_ptr<ov::Model>& model)
{
  Models::BaseModel::updateLayerProperty(model); 
  
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  
  //1. preprocess image inputs
  ov::preprocess::InputInfo& input_info = ppp.input(getInputInfo("input0"));
  ov::Layout tensor_layout = ov::Layout("NHWC");

  if( model->input(0).get_partial_shape().is_dynamic()){
    auto expected_size = getExpectedFrameSize();
    slog::info << "Model's input has dynamic shape, fix it to " << expected_size << slog::endl;
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

  model = ppp.build();
  ppp = ov::preprocess::PrePostProcessor(model);

  ov::Shape input_shape = model->input(getInputInfo("input0")).get_shape();
  slog::debug <<"image_tensor shape is:"<< input_shape.size() <<slog::endl;
  OPENVINO_ASSERT (input_shape.size()== 4);
  setInputHeight(input_shape[1]);
  setInputWidth(input_shape[2]);
  
  //2. Preprocess Outputs
 auto check_output_and_rename = [&](const std::string& output) {
    auto output_info = model->output(output);
    auto shape = output_info.get_partial_shape();
    slog::info << "Output shape for [" << output << "] is: " << shape << slog::endl;
    if (shape.size() == 4){
      slog::info << "find output tensor - [masks]" << slog::endl;
      retagOutputByValue(output, "masks");
    } else if (shape.size() == 3){
      slog::info << "find output tensor - [detection]" << slog::endl;
      retagOutputByValue(output, "detection");
    } else {
      throw std::logic_error("The shape ofoutput tensers are wrong, must be 4 or 3!");
    }
  };
  check_output_and_rename(getOutputName("output0"));
  check_output_and_rename(getOutputName("output1"));

  ov::preprocess::OutputInfo& output_info = ppp.output(getOutputName("masks"));
  output_info.tensor().set_element_type(ov::element::f32);

  model = ppp.build();

  if(model->is_dynamic()) {
    slog::warn << "Model is still dynamic !!!!" << slog::endl;
  } else {
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
  }

  printAttribute();
  slog::info << "Layer Property updated!" << slog::endl;
  return true;

}

bool Models::ObjectSegmentationInstanceModel::fetchResults(
  const std::shared_ptr<Engines::Engine> & engine,
  std::vector<openvino_wrapper_lib::ObjectSegmentationInstanceResult> & results,
  const float & confidence_thresh,
  const bool & enable_roi_constraint)
{

  const float NMS_THRESHOLD = 0.45;   //  threshold for removing overlapping bounding boxes

  ov::InferRequest request = engine->getRequest();
  std::string det_output = getOutputName("detection");
  const ov::Tensor det_output_tensor = request.get_tensor(det_output);
  ov::Shape det_output_shape = det_output_tensor.get_shape();
  auto *detections = det_output_tensor.data<float>();
  int rows = det_output_shape.at(1);
  int dimentions = det_output_shape.at(2);
  cv::Mat output_buffer(det_output_shape[1], det_output_shape[2], CV_32F, detections);
  //Check if transpose is needed
  //if ( needTranspose()){ //DON'T use func needTranspose(), it is not correctly set when calling updateLayerProperty()
  if (det_output_shape.at(2) > det_output_shape.at(1) &&
      det_output_shape.at(2) > 300){ // 300 is just a random number(bigger than the number of classes)
    cv::transpose(output_buffer, output_buffer); //[8400,84+32] for yolov8 seg
    detections = (float*)output_buffer.data;
    rows = det_output_shape.at(2);
    dimentions = det_output_shape.at(1);
  }
  slog::debug << "AFTER calibration: rows->" << rows << ", dimentions->" << dimentions << slog::endl; 

  std::vector<cv::Rect> boxes;
  std::vector<cv::Mat> mask_confs;
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<std::string> & labels = getLabels();

  for (int i = 0; i < rows; i++) {
    //float *detection = &detections[i * dimentions];
    if (hasConfidenceOutput()) {
      float confidence = output_buffer.at<float>(int(i), 4);
      if (confidence < confidence_thresh)
        continue;
    }

    const int classes_scores_start_pos = hasConfidenceOutput()? 5 : 4;
    cv::Mat classes_scores = output_buffer.row(i).colRange(classes_scores_start_pos, dimentions-32); //4, 84

    cv::Point class_id;
    double max_class_score;
    cv::minMaxLoc(classes_scores, nullptr, &max_class_score, nullptr, &class_id);

    if (max_class_score > confidence_thresh) {
        confidences.emplace_back(max_class_score);
        class_ids.emplace_back(class_id.x);

        float x = output_buffer.at<float>(i, 0); //detection[0];
        float y = output_buffer.at<float>(i, 1); //detection[1];
        float w = output_buffer.at<float>(i, 2); //detection[2];
        float h = output_buffer.at<float>(i, 3); //detection[3];
        auto x_min = x - (w / 2);
        auto y_min = y - (h / 2);

        boxes.emplace_back(x_min, y_min, w, h);
        cv::Mat mask_conf = output_buffer.row(i).colRange(dimentions-32, dimentions); //84, 116
        mask_confs.emplace_back(mask_conf);
    }
  }

  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, confidence_thresh, NMS_THRESHOLD, nms_result);

  const ov::Tensor mask_output_tensor = request.get_tensor(getOutputName("masks"));
  ov::Shape mask_output_shape = mask_output_tensor.get_shape();
  //const ov::Layout mask_layout {"NCHW"}; //must be "NCHW"?
  const auto MASK_CHANNEL = mask_output_shape[1];
  const auto MASK_HEIGHT = mask_output_shape[2]; //mask_output_shape[ov::layout::height_idx(mask_layout)];
  const auto MASK_WIDTH = mask_output_shape[3]; //mask_output_shape[ov::layout::width_idx(mask_layout)];
  slog::debug << "mask_output_shape: " << mask_output_shape << ",MASK_HEIGHT:" << MASK_HEIGHT << ", MASK_WIDTH:" << MASK_WIDTH << slog::endl;
  //cv::Mat proto(32, 25600, CV_32F, mask_output_tensor.data<float>()); //[32,25600]
  cv::Mat proto(MASK_CHANNEL, MASK_HEIGHT * MASK_WIDTH, CV_32F, mask_output_tensor.data<float>()); //[32,25600]

  for (int idx: nms_result) {
      double rx = getFrameResizeRatioWidth();
      double ry = getFrameResizeRatioHeight();
      slog::debug << "Detection-Ratio (Input Image to Input Tensor): "<< rx << "x" << ry << slog::endl;

      //Bounding-Box in Input Tensor Size
      int vx = std::max(0, int(boxes[idx].x));
      int vy = std::max(0, int(boxes[idx].y));
      int vw = std::min(std::max(0, int(boxes[idx].width)), getInputWidth()-vx-1);
      int vh = std::min(std::max(0, int(boxes[idx].height)), getInputHeight()-vy-1);

      cv::Rect vrec(vx, vy, vw, vh);
      slog::debug << "Detection Rectangle in Input Tensor Size: " << vrec << slog::endl;
      const int det_bb_x = vx*rx;
      const int det_bb_y = vy*ry;
      const auto frame_size = getFrameSize();
      int det_bb_w = vw*rx;
      int det_bb_h = vh*ry;
      if (det_bb_w+det_bb_x >= frame_size.width){
        det_bb_w = std::max(0, frame_size.width - det_bb_x - 2);
      }
      if (det_bb_h+det_bb_y >= frame_size.height){
        det_bb_h = std::max(0, frame_size.height - det_bb_y -2);
      }
      cv::Rect det_bb(det_bb_x, det_bb_y, det_bb_w, det_bb_h);
      slog::debug << "Detection Rectangle in Input Image Size: " << det_bb << slog::endl;
      Result result(det_bb);
      result.setConfidence(confidences[idx]);
      std::string label = class_ids[idx] < labels.size() ?
        labels[class_ids[idx]] : std::string("label #") + std::to_string(class_ids[idx]);
      result.setLabel(label);

      //Mask data operation
      auto sigmoid = [](float a) {return 1. / (1. + exp(-a));};
      cv::Mat m = mask_confs[idx] * proto;
      for (int col = 0; col < m.cols; col++) {
          m.at<float>(0, col) = sigmoid(m.at<float>(0, col));
      }
      cv::Mat reshaped_m = m.reshape(1, MASK_HEIGHT); //1x25600-->160x160, mask_output_shape:NCHW

      double mask_rx = static_cast<double>(MASK_WIDTH) / getInputWidth();
      double mask_ry = static_cast<double>(MASK_HEIGHT) / getInputHeight();
      slog::debug << "Mask-Ratio (Mask Tensor to Input Tensor): " << mask_rx <<"x" << mask_ry << slog::endl;
      int mask_x = int(mask_rx * vx);
      int mask_y = int(mask_ry * vy);
      int mask_w = int(mask_rx * vw);
      int mask_h = int(mask_ry * vh);
      if (mask_x + mask_w >= MASK_WIDTH){
        mask_w = MASK_WIDTH - 1;
      }
      if (mask_y + mask_h >= MASK_HEIGHT){
        mask_h = MASK_HEIGHT - 1;
      }
      cv::Rect roi{mask_x, mask_y, mask_w, mask_h};
      slog::debug << "Mask ROI:" << roi << slog::endl;
      cv::Mat roi_mask = reshaped_m(roi);
      cv::Mat resized_mask;
      cv::resize(roi_mask, resized_mask, cv::Size(det_bb_w, det_bb_h));
      result.setMask(resized_mask);

      results.push_back(result);
  }

  return true;
}
