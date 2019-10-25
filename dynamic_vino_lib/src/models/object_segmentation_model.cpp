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
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
// Validated Object Detection Network
Models::ObjectSegmentationModel::ObjectSegmentationModel(
  const std::string & model_loc,
  int input_num, int output_num,
  int max_batch_size)
: BaseModel(model_loc, input_num, output_num, max_batch_size)
{
}

void Models::ObjectSegmentationModel::checkNetworkSize(
  int input_size, int output_size, InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info << "Checking input size" << slog::endl;
  InferenceEngine::InputsDataMap input_info(net_reader->getNetwork().getInputsInfo());
  if (input_info.size() != input_size) {
    throw std::logic_error(getModelName() + " should have " + std::to_string(input_size) + " inpu"
            "t, but got " + std::to_string(input_info.size()));
  }

  // check output size
  slog::info << "Checking output size" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(net_reader->getNetwork().getOutputsInfo());
  if (output_info.size() != output_size && output_info.size() != (output_size - 1)) {
    throw std::logic_error(getModelName() + " should have " + std::to_string(output_size) + " outpu"
            "t, but got " + std::to_string(output_info.size()));
  }
}

void Models::ObjectSegmentationModel::setLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
  auto network = net_reader->getNetwork();

  // set input property
  input_info_ = InferenceEngine::InputsDataMap(net_reader->getNetwork().getInputsInfo());
  for (const auto & inputInfoItem : input_info_) {
    if (inputInfoItem.second->getDims().size() == 4) {  // first input contains images
      inputInfoItem.second->setPrecision(InferenceEngine::Precision::U8);
      input_ = inputInfoItem.first; //input_name
    } else if (inputInfoItem.second->getDims().size() == 2) {  // second input contains image info
      inputInfoItem.second->setPrecision(InferenceEngine::Precision::FP32);
    } else {
        throw std::logic_error("Unsupported input shape with size = " + std::to_string(inputInfoItem.second->getDims().size()));
    }
  }

  try {
    network.addOutput(std::string(getDetectionOutputName().c_str()), 0);
  } catch (std::exception & error) {
    throw std::logic_error(getModelName() + "is failed when adding detection_output laryer.");
  }

  network.setBatchSize(1);
  slog::info << "Batch size is " << std::to_string(net_reader->getNetwork().getBatchSize()) <<
    slog::endl;

  input_channels_ = input_info_[input_]->getDims()[2];
  input_height_ = input_info_[input_]->getDims()[1];
  input_width_ = input_info_[input_]->getDims()[0];

  output_info_ = InferenceEngine::OutputsDataMap(net_reader->getNetwork().getOutputsInfo());
  for (auto & item : output_info_) {
    item.second->setPrecision(InferenceEngine::Precision::FP32);
  }

  //Deprecated!
  //auto output_ptr = output_info_.begin();
  //input_ = input_info_map.begin()->first;
  //detection_output_ = output_ptr->first;
  //mask_output_ = (++output_ptr)->first;
}

void Models::ObjectSegmentationModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & net_reader)
{
  const InferenceEngine::CNNLayerPtr output_layer =
    net_reader->getNetwork().getLayerByName(getDetectionOutputName().c_str());
  const int num_classes = output_layer->GetParamAsInt("num_classes");
  slog::info << "Checking Object Segmentation output ... num_classes=" << num_classes << slog::endl;
  if (getLabels().size() != num_classes) {
    if (getLabels().size() == (num_classes - 1)) {
      getLabels().insert(getLabels().begin(), "fake");
    } else {
      getLabels().clear();
    }
  }
}

bool Models::ObjectSegmentationModel::enqueue(
  const std::shared_ptr<Engines::Engine> & engine,
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  if (engine == nullptr) {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  for (const auto & inputInfoItem : input_info_) {
    /** Fill first input tensor with images. First b channel, then g and r channels **/
    if (inputInfoItem.second->getDims().size() == 4) {
      matToBlob(frame, input_frame_loc, 1.0, 0, engine);
    }

    /** Fill second input tensor with image info **/
    if (inputInfoItem.second->getDims().size() == 2) {
      InferenceEngine::Blob::Ptr input = engine->getRequest()->GetBlob(inputInfoItem.first);
      auto data = input->buffer().as<InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type *>();
      data[0] = static_cast<float>(input_height_);  // height
      data[1] = static_cast<float>(input_width_);  // width
      data[2] = 1;
    }
  }
}

bool Models::ObjectSegmentationModel::matToBlob(
  const cv::Mat & orig_image, const cv::Rect &, float scale_factor,
  int batch_index, const std::shared_ptr<Engines::Engine> & engine)
{
  if (engine == nullptr) {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  std::string input_name = getInputName();
  InferenceEngine::Blob::Ptr input_blob =
    engine->getRequest()->GetBlob(input_name);
  auto blob_data = input_blob->buffer().as<InferenceEngine::PrecisionTrait<InferenceEngine::Precision::U8>::value_type *>();

  cv::Mat resized_image(orig_image);
  if(orig_image.size().height != input_height_ || orig_image.size().width != input_width_){
    cv::resize(orig_image, resized_image, cv::Size(input_width_, input_height_));
  }
  int batchOffset = batch_index * input_width_ * input_height_ * input_channels_;

  for (int c = 0; c < input_channels_; c++) {
    for (int h = 0; h < input_height_; h++) {
      for (int w = 0; w < input_width_; w++) {
        blob_data[batchOffset + c * input_width_ * input_height_ + h * input_width_ + w] =
          resized_image.at<cv::Vec3b>(h, w)[c] * scale_factor;
      }
    }
  }

  return true;
}

const std::string Models::ObjectSegmentationModel::getModelName() const
{
  return "Object Segmentation";
}
