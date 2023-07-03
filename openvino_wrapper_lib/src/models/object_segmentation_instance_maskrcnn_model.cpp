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
 * @brief a source code file with declaration of ObjectSegmentationInstanceMaskrcnnModel class
 * It is a child of class ObjectSegmentationInstanceModel.
 */
#include <string>
#include <vector>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/inferences/object_segmentation_instance.hpp"
#include "openvino_wrapper_lib/utils/common.hpp"
#include "openvino_wrapper_lib/models/object_segmentation_instance_maskrcnn_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"

// Validated Object Segmentation Network
Models::ObjectSegmentationInstanceMaskrcnnModel::ObjectSegmentationInstanceMaskrcnnModel(
    const std::string & label_loc,
    const std::string & model_loc,
    int max_batch_size)
    : ObjectSegmentationInstanceModel(label_loc, model_loc, max_batch_size)
{
  setHasConfidenceOutput(true);
  setKeepInputShapeRatio(true);
  setCountOfInputs(2);
  setCountOfOutputs(2);
  setExpectedFrameSize({640, 360});
}

bool Models::ObjectSegmentationInstanceMaskrcnnModel::updateLayerProperty(
    std::shared_ptr<ov::Model>& model)
{
  Models::BaseModel::updateLayerProperty(model);

  slog::debug << "in Models' PrePostProcessor:" << slog::endl;
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  slog::debug << "Model's input size=" << model->inputs().size() << slog::endl;
  //1. preprocess image inputs
  for(int i=0; i<getCountOfInputs(); i++){
    std::string name{"input"};
    name += std::to_string(i);
    slog::debug << "Preprocessing Input: [" << name <<"->" << getInputInfo(name) << slog::endl;
    auto& input_info = ppp.input(getInputInfo(name));
    ov::Layout tensor_layout = ov::Layout("NHWC");
    auto input_shape = model->input(getInputInfo(name)).get_partial_shape();

    if (input_shape.size() == 4) {  // first input contains images
      slog::debug << "handling Input[image_tensor]..." << slog::endl;
      input_info.tensor().
        set_element_type(ov::element::u8).
        set_layout(tensor_layout);
      //addInputInfo(ModelAttribute::DefaultInputName, name);
      //retagInputByValue(getInputInfo(name), "image_tensor");

      if (input_shape.is_dynamic()){
        auto expected_size = getExpectedFrameSize();
        slog::info << "Model's input has dynamic shape, fix it to " << expected_size << slog::endl;
        input_info.tensor().set_shape({1, expected_size.height, expected_size.width, 3}); 
      }
    } else if (input_shape.size() == 2) {  // second input contains image info
      slog::debug << "handling Input[image_info]..." << slog::endl;
      input_info.tensor().set_element_type(ov::element::f32);
      //addInputInfo("input2", info_name_);
      //retagInputByValue(getInputInfo(name), "image_info");
    } else {
      throw std::logic_error("Unsupported input shape with size = " + std::to_string(input_shape.size()));
    }
  }

  //ppp.input(0).model().set_layout("NCHW");

  model = ppp.build();
  ppp = ov::preprocess::PrePostProcessor(model);

  ov::Shape input0_shape = model->input(getInputName("input0")).get_shape();
  slog::debug <<"image_tensor shape is:"<< input0_shape.size() <<slog::endl;
  OPENVINO_ASSERT (input0_shape.size()== 4);
  setInputHeight(input0_shape[1]);
  setInputWidth(input0_shape[2]);
 
  //2. Preprocess Outputs
 auto check_output_and_rename = [&](const std::string& output) {
    auto output_info = model->output(output);
    auto shape = output_info.get_partial_shape();
    slog::info << "Output shape for [" << output << "] is: " << shape << slog::endl;
    if (shape.size() == 4){
      slog::info << "find output tensor - [masks]" << slog::endl;
      retagOutputByValue(output, "masks");
    } else if (shape.size() == 2){
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

bool Models::ObjectSegmentationInstanceMaskrcnnModel::fetchResults(
  const std::shared_ptr<Engines::Engine> & engine,
  std::vector<openvino_wrapper_lib::ObjectSegmentationInstanceResult> & results,
  const float & confidence_thresh,
  const bool & enable_roi_constraint)
{
  ov::InferRequest infer_request = engine->getRequest();
  slog::debug << "Analyzing Detection results..." << slog::endl;
  std::string detection_output = getOutputName("detection");
  std::string mask_output = getOutputName("masks");
  slog::debug << "Detection_output=" << detection_output << ", Mask_output=" << mask_output << slog::endl;
 
  //get detection data
  ov::Tensor do_tensor = infer_request.get_tensor(detection_output);
  const auto do_data = do_tensor.data<float>();
  ov::Shape do_shape = do_tensor.get_shape();
  slog::debug << "Detection Blob getDims = " <<do_shape.size() << "[Should be 2]" << slog::endl;
  // get mask data
  ov::Tensor mask_tensor = infer_request.get_tensor(mask_output);
  const auto mask_data = mask_tensor.data<float>();
  ov::Shape mask_shape = mask_tensor.get_shape();

  // determine models
  size_t box_description_size = do_shape.back();
  OPENVINO_ASSERT(mask_shape.size() == 4);
  size_t box_num = mask_shape[0];
  size_t C = mask_shape[1];
  size_t H = mask_shape[2];
  size_t W = mask_shape[3];
  size_t box_stride = W * H * C;
  slog::debug << "box_description is:" << box_description_size << slog::endl;
  slog::debug << "box_num is:" << box_num<< slog::endl;
  slog::debug << "C is:" << C << slog::endl;
  slog::debug << "H is:" << H << slog::endl;
  slog::debug << "W is:" << W << slog::endl;

  for (size_t box = 0; box < box_num; ++box) {
    // box description: batch, label, prob, x1, y1, x2, y2
    float * box_info = do_data + box * box_description_size;
    auto batch = static_cast<int>(box_info[0]);
    slog::debug << "batch =" << batch << slog::endl;
    if (batch < 0) {
      slog::warn << "Batch size should be greater than 0. [batch=" << batch <<"]." << slog::endl;
      break;
    }
    float prob = box_info[2];
    const double rx = getFrameResizeRatioWidth();
    const double ry = getFrameResizeRatioHeight();
    //slog::debug << "FrameResizeRatio W:" <<rx << ", H:" << ry << slog::endl;
    slog::debug << "Got an object with probability:" << prob << ", threshold:" << confidence_thresh << slog::endl;
    auto iW=getInputWidth();
    auto iH=getInputHeight();
    if (prob > confidence_thresh) {
      float x1 = std::min(std::max(0.0f, box_info[3] * iW), static_cast<float>(iW)) * rx;
      float y1 = std::min(std::max(0.0f, box_info[4] * iH), static_cast<float>(iH)) * ry;
      float x2 = std::min(std::max(0.0f, box_info[5] * iW), static_cast<float>(iW)) * rx;
      float y2 = std::min(std::max(0.0f, box_info[6] * iH), static_cast<float>(iH)) * ry;
      auto fSize = getFrameSize();
      if ((int)x2 >= fSize.width){
        x2 = fSize.width-2;
      }
      if ((int)y2 >= fSize.height){
        x2 = fSize.height-2;
      }
      int box_width = static_cast<int>(x2 - x1);
      int box_height = static_cast<int>(y2 - y1);
      slog::debug << "Box[" << box_width << "x" << box_height << "]" << slog::endl;
      if (box_width <= 0 || box_height <=0) break;
      int class_id = static_cast<int>(box_info[1] + 1e-6f);
      float * mask_arr = mask_data + box_stride * box + H * W * (class_id - 1);
      cv::Mat mask_mat(H, W, CV_32FC1, mask_arr);
      cv::Rect roi = cv::Rect(static_cast<int>(x1), static_cast<int>(y1), box_width, box_height)
                     /*& cv::Rect({0, 0}, getFrameSize()-cv::Size{2, 2})*/;
      slog::info << "Detected class " << class_id << " with probability " << prob << " from batch " << batch
                          << ": " << roi << slog::endl;
      cv::Mat resized_mask_mat(box_height, box_width, CV_32FC1);
      cv::resize(mask_mat, resized_mask_mat, cv::Size(box_width, box_height));
      Result result(roi);
      result.setConfidence(prob);
      std::vector<std::string> & labels = getLabels();
      std::string label = class_id < labels.size() ? labels[class_id] :
        std::string("label #") + std::to_string(class_id);
      result.setLabel(label);
      result.setMask(resized_mask_mat);
      slog::debug << "adding one segmentation Box ..." << slog::endl;
      results.emplace_back(result);
    }
  }

  return true;
}
