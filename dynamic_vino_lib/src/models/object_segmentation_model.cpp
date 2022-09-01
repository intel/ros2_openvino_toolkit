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
#include <inference_engine.hpp>
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/utils/common.hpp"
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

  for (const auto &inputInfoItem : input_info_)
  {
    // Fill first input tensor with images. First b channel, then g and r channels
    slog::debug<<"first tensor"<<inputInfoItem.second->getTensorDesc().getDims().size()<<slog::endl;
    if (inputInfoItem.second->getTensorDesc().getDims().size()==4)
    {
      matToBlob(frame, input_frame_loc, 1.0, 0, engine);
    }

    // Fill second input tensor with image info
    if (inputInfoItem.second->getTensorDesc().getDims().size() == 2)
    {
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

  InferenceEngine::Blob::Ptr blob = engine->getRequest()->GetBlob(getInputName("input"));
  InferenceEngine::SizeVector blobSize = blob->getTensorDesc().getDims();
  IE_ASSERT(blobSize.size()== 4);
  const size_t width = blobSize[3];
  const size_t height = blobSize[2];
  const size_t channels = blobSize[1];
  if (static_cast<size_t>(orig_image.channels()) != channels) {
     throw std::runtime_error("The number of channels for net input and image must match");
  }
  
  InferenceEngine::LockedMemory<void> blobMapped = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob)->wmap();
  unsigned char* blob_data = blobMapped.as<unsigned char*>();

  cv::Mat resized_image(orig_image);
  if (static_cast<int>(width) != orig_image.size().width ||
          static_cast<int>(height) != orig_image.size().height) {
      cv::resize(orig_image, resized_image, cv::Size(width, height));
  }

  int batchOffset = batch_index * width * height * channels;

  if (channels == 1) {
      for (size_t  h = 0; h < height; h++) {
          for (size_t w = 0; w < width; w++) {
              blob_data[batchOffset + h * width + w] = resized_image.at<uchar>(h, w);
          }
      }
  } else if (channels == 3) {
      for (size_t c = 0; c < channels; c++) {
          for (size_t  h = 0; h < height; h++) {
              for (size_t w = 0; w < width; w++) {
                  blob_data[batchOffset + c * width * height + h * width + w] =
                          resized_image.at<cv::Vec3b>(h, w)[c];
              }
          }
      }
  } else {
      throw std::runtime_error("Unsupported number of channels");
  }

  return true;
}

const std::string Models::ObjectSegmentationModel::getModelCategory() const
{
  return "Object Segmentation";
}

bool Models::ObjectSegmentationModel::updateLayerProperty(
    InferenceEngine::CNNNetwork& net_reader)
{
  slog::info<< "Checking INPUTS for Model" <<getModelName()<<slog::endl;

  auto network = net_reader;

  try {
    network.addOutput(std::string("detection_output"), 0);
  } catch (std::exception & error) {
    throw std::logic_error(getModelName() + "is failed when adding detection_output laryer.");
  }

  input_info_ = InferenceEngine::InputsDataMap(network.getInputsInfo());
  for (const auto & inputInfoItem : input_info_) {
    if (inputInfoItem.second->getTensorDesc().getDims().size() == 4) {  // first input contains images
      addInputInfo("input", inputInfoItem.first);
      inputInfoItem.second->setPrecision(InferenceEngine::Precision::U8);
    } else if (inputInfoItem.second->getTensorDesc().getDims().size() == 2) {  // second input contains image info
      addInputInfo("input2", inputInfoItem.first);
      inputInfoItem.second->setPrecision(InferenceEngine::Precision::FP32);
    } else {
      throw std::logic_error("Unsupported input shape with size = " + std::to_string(inputInfoItem.second->getTensorDesc().getDims().size()));
    }
  }

  /** network dimensions for image input **/
  std::string inputName = getInputName("input");
  slog::debug << "Model's input name is " << inputName << slog::endl;
  const InferenceEngine::TensorDesc& inputDesc = input_info_[inputName]->getTensorDesc();
  IE_ASSERT(inputDesc.getDims().size() == 4);
  size_t netBatchSize = getTensorBatch(inputDesc);
  size_t netInputHeight = getTensorHeight(inputDesc);
  size_t netInputWidth = getTensorWidth(inputDesc);
  slog::debug << "neetBatchSize=" << netBatchSize << ", netInputHeight=" << netInputHeight << ", netInputWidth=" << netInputWidth << slog::endl;

  InferenceEngine::OutputsDataMap outputsDataMap = network.getOutputsInfo();
  slog::debug<<"The size of Outputs Datamap is "<< outputsDataMap.size() <<slog::endl;
  for (auto & output_item : outputsDataMap) {
    output_item.second->setPrecision(InferenceEngine::Precision::FP32);
  }

  auto it = outputsDataMap.begin();
  addOutputInfo("detection", it->first);
  slog::debug << "Detection_Output is set to " << it->first << slog::endl;
  it++;
  addOutputInfo("masks", it->first);
  slog::debug << "Mask_output is set to " << it->first <<slog::endl;

  printAttribute();
  slog::info << "This model Layer Property updated!" << slog::endl;
  return true;
}
