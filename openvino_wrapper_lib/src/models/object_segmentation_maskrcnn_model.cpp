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
 * @brief a header file with declaration of ObjectSegmentationModel class
 * @file object_segmentation_model.cpp
 */
#include <string>
#include <vector>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/models/object_segmentation_maskrcnn_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"

// Validated Object Segmentation Network
Models::ObjectSegmentationMaskrcnnModel::ObjectSegmentationMaskrcnnModel(
    const std::string & label_loc, 
    const std::string & model_loc,
    int max_batch_size)
    : BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::ObjectSegmentationMaskrcnnModel::enqueue(
    const std::shared_ptr<Engines::Engine> &engine,
    const cv::Mat &frame,
    const cv::Rect &input_frame_loc)
{
  if (engine == nullptr)
  {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  for (const auto &inputInfoItem : inputs_info_)
  {
    // Fill first input tensor with images. First b channel, then g and r channels
    auto dims = inputInfoItem.get_shape();
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

bool Models::ObjectSegmentationMaskrcnnModel::matToBlob(
    const cv::Mat &orig_image, const cv::Rect &, float scale_factor,
    int batch_index, const std::shared_ptr<Engines::Engine> &engine)
{
  (void)scale_factor;
  (void)batch_index;

  if (engine == nullptr)
  {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  ov::InferRequest infer_request = engine->getRequest();
  ov::Tensor input_tensor = infer_request.get_tensor(getInputName());
  ov::Shape input_shape = input_tensor.get_shape();

  OPENVINO_ASSERT(input_shape.size() == 4);
  // For frozen graph model: layout= "NHWC"
  const size_t width = input_shape[2];
  const size_t height = input_shape[1];
  const size_t channels = input_shape[3];

  slog::debug <<"width is:"<< width << slog::endl;
  slog::debug <<"height is:"<< height << slog::endl;
  slog::debug <<"channels is:"<< channels << slog::endl;
  slog::debug <<"origin channels is:"<< orig_image.channels() << slog::endl;
  slog::debug <<"input shape is:"<< input_shape << slog::endl;

  if (static_cast<size_t>(orig_image.channels()) != channels) {
     throw std::runtime_error("The number of channels for net input and image must match");
  }
  
#if 1
  //input_tensor = ov::Tensor(ov::element::u8, {1, height, width, channels}, resized_image.data);
  //engine->getRequest().set_tensor(input_tensor_name_, input_tensor);
  unsigned char* data = input_tensor.data<unsigned char>();
  cv::Size size = {(int)width, (int)height};
  cv::Mat resized_image(size, CV_8UC3, data);
  cv::resize(orig_image, resized_image, size);
#else
  const auto input_data = input_tensor.data<unsigned char>();
  cv::Mat resized_image(orig_image);
  if (static_cast<int>(width) != orig_image.size().width ||
          static_cast<int>(height) != orig_image.size().height) {
      cv::resize(orig_image, resized_image, cv::Size(width, height));
  }

  int batchOffset = batch_index * width * height * channels;
  if (channels == 1) {
      for (size_t  h = 0; h < height; h++) {
          for (size_t w = 0; w < width; w++) {
              input_data[batchOffset + h * width + w] = resized_image.at<uchar>(h, w);
          }
      }
  } else if (channels == 3) {
      for (size_t c = 0; c < channels; c++) {
          for (size_t  h = 0; h < height; h++) {
              for (size_t w = 0; w < width; w++) {
                  input_data[batchOffset + c * width * height + h * width + w] =
                          resized_image.at<cv::Vec3b>(h, w)[c];
              }
          }
      }
  } else {
      throw std::runtime_error("Unsupported number of channels");
  }
#endif

  return true;
}

const std::string Models::ObjectSegmentationMaskrcnnModel::getModelCategory() const
{
  return "Object Segmentation";
}

bool Models::ObjectSegmentationMaskrcnnModel::updateLayerProperty(
    std::shared_ptr<ov::Model>& model)
{
  slog::info<< "Checking INPUTS for Model" <<getModelName()<<slog::endl;
  
  // check input shape
  inputs_info_ = model->inputs();
  slog::debug<<"input size="<<inputs_info_.size()<<slog::endl;
  if (inputs_info_.size() != 2) {
    slog::warn << "This inference sample should have have two inputs, but we got"
      << std::to_string(inputs_info_.size()) << "inputs"
      << slog::endl;
    return false;
  }
  
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = inputs_info_[0].get_any_name();
  auto info_name_ = inputs_info_[1].get_any_name();
  slog::debug<<"input_tensor_name is:"<<input_tensor_name_<<slog::endl;
  slog::debug<<"input_info_name is:"<<info_name_<<slog::endl;
  
  // preprocess image inputs
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);
  ov::Layout tensor_layout = ov::Layout("NHWC");
  ov::Shape input_shape = model->input("image_tensor").get_shape();
  slog::debug<<"image_tensor shape is:"<< input_shape.size() <<slog::endl;

  // preprocess image info inputs
  ov::preprocess::InputInfo & image_info = ppp.input(info_name_);
  ov::Layout info_layout = ov::Layout("NC");
  ov::Shape info_shape = model->input("image_info").get_shape();
  slog::debug<<"image_info shape is:"<< info_shape.size() <<slog::endl;

  for (const auto & inputInfoItem : inputs_info_){
    if (input_shape.size() == 4) {  // first input contains images
      input_info.tensor().
      set_element_type(ov::element::u8).
      set_layout(tensor_layout);
      addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);
    } else if (info_shape.size() == 2) {  // second input contains image info
      image_info.tensor().set_element_type(ov::element::f32);
      addInputInfo("input2", info_name_);
    } else {
      throw std::logic_error("Unsupported input shape with size = " + std::to_string(input_shape.size()));
    }
  }

  std::string inputName = getInputName();
  slog::debug << "input name is:" << inputName << slog::endl;
  OPENVINO_ASSERT (input_shape.size()== 4);
  size_t netBatchSize = input_shape[0];
  size_t netInputHeight = input_shape[1];
  size_t netInputWidth = input_shape[2];
  slog::debug << "netBatchSize=" << netBatchSize << ", netInputHeight=" << netInputHeight << ", netInputWidth=" << netInputWidth << slog::endl;
  
  // check output shape
  outputs_info_ = model->outputs();
  slog::debug<<"output size="<<outputs_info_.size()<<slog::endl;
  if (outputs_info_.size() != 2) {
    slog::warn << "This inference sample should have have 2 outputs, but we got"
      << std::to_string(outputs_info_.size()) << "outputs"
      << slog::endl;
    return false;
  }

  // preprocess outshape
  output_tensor_name_ = outputs_info_[0].get_any_name();
  auto detection_name_ = outputs_info_[1].get_any_name();
  slog::debug<<"output_tensor_name is:"<<output_tensor_name_<<slog::endl;
  slog::debug<<"detection_name_is:"<<detection_name_<<slog::endl;

  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  ov::Shape mask_shape = model->output("masks").get_shape();
  slog::debug<<"masks shape is:"<< mask_shape.size() <<slog::endl;
  ov::Shape detection_shape = model->output("reshape_do_2d").get_shape();
  slog::debug<< "detection shape is:" << detection_shape.size() <<slog::endl;
  output_info.tensor().set_element_type(ov::element::f32);

  model = ppp.build();

  addOutputInfo("masks", output_tensor_name_);
  slog::debug << "Mask_Output is set to " << output_tensor_name_ << slog::endl;
  addOutputInfo("detection", detection_name_);
  slog::debug << "Detection_Output is set to " <<detection_name_<< slog::endl;

  printAttribute();
  slog::info << "This model is SSDNet-like, Layer Property updated!" << slog::endl;
  return true;

}

