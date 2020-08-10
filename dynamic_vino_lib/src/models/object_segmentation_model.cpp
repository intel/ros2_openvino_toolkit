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
#include <vector>
#include <inference_engine.hpp>
#include "dynamic_vino_lib/models/object_segmentation_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
// Validated Object Detection Network
Models::ObjectSegmentationModel::ObjectSegmentationModel(
    const std::string &model_loc,
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
  if (input_info.size() != input_size)
  {
    throw std::logic_error(getModelName() + " should have " + std::to_string(input_size) + " inpu"
                                                                                           "t, but got " +
                           std::to_string(input_info.size()));
  }

  // check output size
  slog::info << "Checking output size" << slog::endl;
  InferenceEngine::OutputsDataMap output_info(net_reader->getNetwork().getOutputsInfo());
  if (output_info.size() != output_size && output_info.size() != (output_size - 1))
  {
    throw std::logic_error(getModelName() + " should have " + std::to_string(output_size) + " outpu"
                                                                                            "t, but got " +
                           std::to_string(output_info.size()));
  }
}

void Models::ObjectSegmentationModel::setLayerProperty(
    InferenceEngine::CNNNetReader::Ptr net_reader)
{
  auto network = net_reader->getNetwork();

  // set input property
  auto inputShapes = network.getInputShapes();
  if (inputShapes.size() != 1)
    throw std::runtime_error("Demo supports topologies only with 1 input");
  input_ = inputShapes.begin()->first;
  InferenceEngine::SizeVector &in_size_vector = inputShapes.begin()->second;
  if (in_size_vector.size() != 4 || in_size_vector[1] != 3)
    throw std::runtime_error("3-channel 4-dimensional model's input is expected");
  in_size_vector[0] = 1; // set batch size to 1
  network.reshape(inputShapes);

  auto &inputInfo = *network.getInputsInfo().begin()->second;
  inputInfo.getPreProcess().setResizeAlgorithm(InferenceEngine::ResizeAlgorithm::RESIZE_BILINEAR);
  inputInfo.setLayout(InferenceEngine::Layout::NHWC);
  inputInfo.setPrecision(InferenceEngine::Precision::U8);

  try
  {
    network.addOutput(std::string(getDetectionOutputName().c_str()), 0);
  }
  catch (std::exception &error)
  {
    throw std::logic_error(getModelName() + "is failed when adding detection_output laryer.");
  }

  network.setBatchSize(1);
  slog::info << "Batch size is " << std::to_string(net_reader->getNetwork().getBatchSize()) << slog::endl;

  output_info_ = InferenceEngine::OutputsDataMap(net_reader->getNetwork().getOutputsInfo());

  for (auto &item : output_info_)
  {
    item.second->setPrecision(InferenceEngine::Precision::FP32);
    const InferenceEngine::SizeVector& out_size_vector = item.second->getTensorDesc().getDims();
    if (item.first == getMaskOutputName()){
      switch(out_size_vector.size()){
        case 3:
          output_channels_ = 0;
          output_height_ = out_size_vector[1];
          output_width_ = out_size_vector[2];
          break;
        case 4:
          output_channels_ = out_size_vector[1];
          output_height_ = out_size_vector[2];
          output_width_ = out_size_vector[3];
          break;
        default:
          throw std::runtime_error("Unexpected output blob shape. Only 4D and 3D output blobs are"
            "supported.");
      }
    }
  }

  if(output_height_ == 0 || output_width_ == 0){
    slog::err << "output_height or output_width is not set, please check the MaskOutput Info "
              << "is set correctly." << slog::endl;
    throw std::runtime_error("output_height or output_width is not set, please check the MaskOutputInfo");
  }

  //Deprecated!
  //auto output_ptr = output_info_.begin();
  //input_ = input_info_map.begin()->first;
  //detection_output_ = output_ptr->first;
  //mask_output_ = (++output_ptr)->first;
}

void Models::ObjectSegmentationModel::checkLayerProperty(
    const InferenceEngine::CNNNetReader::Ptr &net_reader)
{
  const InferenceEngine::CNNLayerPtr output_layer =
      net_reader->getNetwork().getLayerByName(getDetectionOutputName().c_str());
  const int num_classes = output_layer->GetParamAsInt("num_classes");
  slog::info << "Checking Object Segmentation output ... num_classes=" << num_classes << slog::endl;
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
    /** Fill first input tensor with images. First b channel, then g and r channels **/
    if (inputInfoItem.second->getDims().size() == 4)
    {
      matToBlob(frame, input_frame_loc, 1.0, 0, engine);
    }

    /** Fill second input tensor with image info **/
    if (inputInfoItem.second->getDims().size() == 2)
    {
      InferenceEngine::Blob::Ptr input = engine->getRequest()->GetBlob(inputInfoItem.first);
      auto data = input->buffer().as<InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type *>();
      data[0] = static_cast<float>(frame.rows); // height
      data[1] = static_cast<float>(frame.cols);  // width
      data[2] = 1;
    }
  }
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

const std::string Models::ObjectSegmentationModel::getModelName() const
{
  return "Object Segmentation";
}
