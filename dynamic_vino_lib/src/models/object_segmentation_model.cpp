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
 * @brief a header file with declaration of ObjectSegmentationModel class
 * @file object_segmentation_model.cpp
 */
#include <string>
#include <vector>
#include <openvino/openvino.hpp>
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
// Validated Object Segmentation Network
Models::ObjectSegmentationModel::ObjectSegmentationModel(
    const std::string & label_loc, 
    const std::string & model_loc,
    int max_batch_size)
    : BaseModel(label_loc, model_loc, max_batch_size)
{
}

bool Models::ObjectSegmentationModel::enqueue(
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

bool Models::ObjectSegmentationModel::matToBlob(
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

  size_t channels = orig_image.channels();
  size_t height = orig_image.size().height;
  size_t width = orig_image.size().width;

  size_t strideH = orig_image.step.buf[0];
  size_t strideW = orig_image.step.buf[1];

  bool is_dense =
      strideW == channels &&
      strideH == channels * width;

  if (!is_dense){
    slog::err << "Doesn't support conversion from not dense cv::Mat." << slog::endl;
    return false;
  }

  ov::Tensor input_tensor = ov::Tensor(ov::element::u8, {1, height, width, channels}, orig_image.data);
  engine->getRequest().set_tensor(input_tensor_name_, input_tensor);

  return true;
}

const std::string Models::ObjectSegmentationModel::getModelCategory() const
{
  return "Object Segmentation";
}

bool Models::ObjectSegmentationModel::updateLayerProperty(
    std::shared_ptr<ov::Model>& model)
{
  slog::info<< "Checking INPUTS for Model" <<getModelName()<<slog::endl;

  inputs_info_ = model->inputs();
  slog::debug<<"input size"<<inputs_info_.size()<<slog::endl;
  if (inputs_info_.size() != 1) {
    slog::warn << "This inference sample should have only one input, but we got"
      << std::to_string(inputs_info_.size()) << "inputs"
      << slog::endl;
    return false;
  }
  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);

  ov::Layout tensor_layout = ov::Layout("NHWC");
  ov::Layout expect_layout = ov::Layout("NCHW");
  ov::Shape input_shape = model->input().get_shape();
  if (input_shape[1] == 3)
    expect_layout = ov::Layout("NCHW");
  else if (input_shape[3] == 3)
    expect_layout = ov::Layout("NHWC");
  else
    slog::warn << "unexpect input shape " << input_shape << slog::endl;

  input_info.tensor().
    set_element_type(ov::element::u8).
    set_layout(tensor_layout).
    set_spatial_dynamic_shape();
  input_info.preprocess().
    convert_layout(expect_layout).
    resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR);
  addInputInfo("input", input_tensor_name_);

  auto outputs_info = model->outputs();
  if (outputs_info.size() != 1) {
    slog::warn << "This inference sample should have only one output, but we got"
      << std::to_string(outputs_info.size()) << "outputs"
      << slog::endl;
    return false;
  }

  output_tensor_name_ = model->output().get_any_name();
  auto data = model->output();

  ov::preprocess::OutputInfo& output_info = ppp.output(output_tensor_name_);
  output_info.tensor().set_element_type(ov::element::f32);
  model = ppp.build();
  std::vector<size_t> &in_size_vector = input_shape;
  slog::debug<<"dimensional"<<in_size_vector.size()<<slog::endl;
  if (in_size_vector.size() != 4) {
    slog::warn << "3-channel 4-dimensional model's input is expected, but we got "
      << std::to_string(in_size_vector.size()) << " dimensions." << slog::endl;
    return false;
  }

  auto& outSizeVector = data.get_shape();
  int outChannels, outHeight, outWidth;
  slog::debug << "output size vector " << outSizeVector.size() << slog::endl;
  switch(outSizeVector.size()){
    case 3:
      outChannels = 0;
      outHeight = outSizeVector[1];
      outWidth = outSizeVector[2];
      break;
    case 4:
      outChannels = outSizeVector[1];
      outHeight = outSizeVector[2];
      outWidth = outSizeVector[3];
      break;
    default:
      throw std::runtime_error("Unexpected output blob shape. Only 4D and 3D output blobs are"
                    "supported.");

  }
  if(outHeight == 0 || outWidth == 0){
    slog::err << "output_height or output_width is not set, please check the MaskOutput Info "
              << "is set correctly." << slog::endl;
    return false;
  }

  slog::debug << "output width " << outWidth<< slog::endl;
  slog::debug << "output HEIGHT " << outHeight<< slog::endl;
  slog::debug << "output CHANNALS " << outChannels<< slog::endl;
  slog::debug << "output name " << output_tensor_name_<< slog::endl;
  addOutputInfo("masks", output_tensor_name_);
  addOutputInfo("detection", output_tensor_name_);

  printAttribute();
  slog::info << "This model is SSDNet-like, Layer Property updated!" << slog::endl;
  return true;

}
