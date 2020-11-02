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
 * @file object_detection_model.cpp
 */
#include <string>
#include <memory>
#include <vector>
#include <inference_engine.hpp>
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/models/attributes/base_attribute.hpp"
// Validated Object Detection Network
Models::ObjectSegmentationModel::ObjectSegmentationModel(
    const std::string &model_loc,
    int max_batch_size)
    : BaseModel(model_loc, max_batch_size)
{
}

bool Models::ObjectSegmentationModel::enqueue(
    const std::shared_ptr<Engines::Engine> &engine,
    const cv::Mat &frame,
    const cv::Rect &input_frame_loc)
{
  slog::debug<<"enque is beginning" <<slog::endl;
  if (engine == nullptr)
  {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }
    
  for (const auto &inputInfoItem : input_info_)
  {
    // Fill first input tensor with images. First b channel, then g and r channels 
    slog::debug<<"first tensor"<<inputInfoItem.second->getTensorDesc().getDims().size()<<slog::endl;
    if (inputInfoItem.second->getTensorDesc().getDims().size()==4)
    {
      slog::debug << "Fill first input tensor with images" <<slog::endl;
      matToBlob(frame, input_frame_loc, 1.0, 0, engine);
    }

    // Fill second input tensor with image info 
    //if (inputInfoItem.second->getDims().size() == 2)
    if (inputInfoItem.second->getTensorDesc().getDims().size() == 2)
    {
      slog::debug << "Fill second input tensor with image info" <<slog::endl;
      InferenceEngine::Blob::Ptr input = engine->getRequest()->GetBlob(inputInfoItem.first);
      auto data = input->buffer().as<InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type *>();
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
  
  InferenceEngine::TensorDesc tDesc(InferenceEngine::Precision::U8,
                                    {1, channels, height, width},
                                    InferenceEngine::Layout::NHWC);
  

  auto shared_blob = InferenceEngine::make_shared_blob<uint8_t>(tDesc, orig_image.data);
  engine->getRequest()->SetBlob(getInputName(), shared_blob);
  
  return true;
}

const std::string Models::ObjectSegmentationModel::getModelCategory() const
{
  return "Object Segmentation";
}

bool Models::ObjectSegmentationModel::updateLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info<< "Checking INPUTS for Model" <<getModelName()<<slog::endl;

  auto network = net_reader->getNetwork();
  
  input_info_ = InferenceEngine::InputsDataMap(network.getInputsInfo());

  InferenceEngine::ICNNNetwork:: InputShapes inputShapes = network.getInputShapes();
  slog::debug<<"input size"<<inputShapes.size()<<slog::endl;
  if (inputShapes.size() != 1)
    throw std::runtime_error("Demo supports topologies only with 1 input");
  InferenceEngine::SizeVector &in_size_vector = inputShapes.begin()->second;
  const std:: string& inName = inputShapes.begin()->first;
  //slog::debug << "input name" << inName << slog::endl;

  slog::debug<<"channel size"<<in_size_vector[1]<<"dimensional"<<in_size_vector.size()<<slog::endl;
  if (in_size_vector.size() != 4 || in_size_vector[1] != 3)
    throw std::runtime_error("3-channel 4-dimensional model's input is expected");
  in_size_vector[0] = 1;
  network.reshape(inputShapes);

  InferenceEngine:: InputInfo &inputInfo = *network.getInputsInfo().begin()->second;
  inputInfo.getPreProcess().setResizeAlgorithm(InferenceEngine::ResizeAlgorithm::RESIZE_BILINEAR);
  inputInfo.setLayout(InferenceEngine::Layout::NHWC);
  inputInfo.setPrecision(InferenceEngine::Precision::U8);
  //input_info_ = inputInfo;

  InferenceEngine:: InputsDataMap input_info_map(network.getInputsInfo());
  if (input_info_map.size() != 1){
    slog::warn << "This model seems not SSDNet-like, SSDnet has only one input, but we got "
      << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    return false;
  }
  //InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  //addInputInfo("input", input_info_map.begin()->first.c_str());
  addInputInfo("input", inputShapes.begin()->first);

  InferenceEngine::OutputsDataMap outputsDataMap = network.getOutputsInfo();
  if (outputsDataMap.size() != 1) 
    throw std::runtime_error("Demo supports topologies only with 1 output");
  const std::string& outName = outputsDataMap.begin()->first;
  InferenceEngine::Data & data = *outputsDataMap.begin()->second;
  data.setPrecision(InferenceEngine::Precision::FP32);

  const InferenceEngine::SizeVector& outSizeVector = data.getTensorDesc().getDims();
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
    throw std::runtime_error("output_height or output_width is not set, please check the MaskOutputInfo");
  }
  
  slog::debug << "output width " << outWidth<< slog::endl;
  slog::debug << "output hEIGHT " << outHeight<< slog::endl;
  slog::debug << "output CHANNALS " << outChannels<< slog::endl;
  //slog::debug << "mask name" << inName << slog::endl;
  addOutputInfo("masks", (outputsDataMap.begin()++)->first);
  addOutputInfo("detection", outputsDataMap.begin()->first);

  //const InferenceEngine::CNNLayerPtr output_layer =
      //network.getLayerByName(outputsDataMap.begin()->first.c_str());
  const InferenceEngine::CNNLayerPtr output_layer = 
      network.getLayerByName(getOutputName("detection").c_str());
  //const int num_classes = output_layer->GetParamAsInt("num_classes");
  //slog::info << "Checking Object Segmentation output ... num_classes=" << num_classes << slog::endl;

#if 0
  if (getLabels().size() != num_classes)
  {
    if (getLabels().size() == (num_classes - 1))
    {
      getLabels().insert(getLabels().begin(), "fake");
    }
    else
    {
      getLabels().clear();
    }
  }
#endif
/*
  const InferenceEngine::SizeVector output_dims = data.getTensorDesc().getDims();
  setMaxProposalCount(static_cast<int>(output_dims[2]));
  slog::info << "max proposal count is: " << getMaxProposalCount() << slog::endl;
  auto object_size = static_cast<int>(output_dims[3]);
  setObjectSize(object_size);

  slog::debug << "model size" << output_dims.size() << slog::endl;*/
  printAttribute();
  slog::info << "This model is SSDNet-like, Layer Property updated!" << slog::endl;
  return true;

}
