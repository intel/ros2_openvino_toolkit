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
 * @brief a header file with declaration of ObjectDetectionSSDModel class
 * @file object_detection_model.cpp
 */
#include <string>
#include <memory>
#include <vector>
#include "dynamic_vino_lib/inferences/object_detection.hpp"
#include "dynamic_vino_lib/models/object_detection_ssd_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"

// Validated Object Detection Network
Models::ObjectDetectionSSDModel::ObjectDetectionSSDModel(
  const std::string & model_loc, int input_num,
  int output_num, int max_batch_size)
: ObjectDetectionModel(model_loc, input_num, output_num, max_batch_size)
{
}

void Models::ObjectDetectionSSDModel::setLayerProperty(
  InferenceEngine::CNNNetReader::Ptr net_reader)
{
  // set input property
  InferenceEngine::InputsDataMap input_info_map(net_reader->getNetwork().getInputsInfo());
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  input_info->getInputData()->setLayout(InferenceEngine::Layout::NCHW);
  // set output property
  InferenceEngine::OutputsDataMap output_info_map(net_reader->getNetwork().getOutputsInfo());
  InferenceEngine::DataPtr & output_data_ptr = output_info_map.begin()->second;
  output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  output_data_ptr->setLayout(InferenceEngine::Layout::NCHW);
  // set input and output layer name
  input_ = input_info_map.begin()->first;
  output_ = output_info_map.begin()->first;
}

void Models::ObjectDetectionSSDModel::checkLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr & net_reader)
{
  slog::info << "Checking Object Detection outputs" << slog::endl;
  InferenceEngine::OutputsDataMap output_info_map(net_reader->getNetwork().getOutputsInfo());
  slog::info << "Checking Object Detection outputs ..." << slog::endl;
  if (output_info_map.size() != 1) {
    throw std::logic_error("This sample accepts networks having only one output");
  }
  InferenceEngine::DataPtr & output_data_ptr = output_info_map.begin()->second;
  output_ = output_info_map.begin()->first;
  slog::info << "Checking Object Detection output ... Name=" << output_ << slog::endl;

  const InferenceEngine::CNNLayerPtr output_layer =
    net_reader->getNetwork().getLayerByName(output_.c_str());
  // output layer should have attribute called num_classes
  slog::info << "Checking Object Detection num_classes" << slog::endl;
  if (output_layer->params.find("num_classes") == output_layer->params.end()) {
    throw std::logic_error("Object Detection network output layer (" + output_ +
            ") should have num_classes integer attribute");
  }
  // class number should be equal to size of label vector
  // if network has default "background" class, fake is used
  const int num_classes = output_layer->GetParamAsInt("num_classes");

  slog::info << "Checking Object Detection output ... num_classes=" << num_classes << slog::endl;
  if (getLabels().size() != num_classes) {
    if (getLabels().size() == (num_classes - 1)) {
      getLabels().insert(getLabels().begin(), "fake");
    } else {
      getLabels().clear();
    }
  }
  // last dimension of output layer should be 7
  const InferenceEngine::SizeVector output_dims = output_data_ptr->getTensorDesc().getDims();
  max_proposal_count_ = static_cast<int>(output_dims[2]);
  slog::info << "max proposal count is: " << max_proposal_count_ << slog::endl;

  auto object_size = static_cast<int>(output_dims[3]);
  if (object_size != 7) {
    throw std::logic_error("Object Detection network output layer should have 7 as a last "
            "dimension");
  }
  setObjectSize(object_size);

  if (output_dims.size() != 4) {
    throw std::logic_error("Object Detection network output dimensions not compatible shoulld be "
            "4, "
            "but was " +
            std::to_string(output_dims.size()));
  }
}

const std::string Models::ObjectDetectionSSDModel::getModelName() const
{
  return "Object Detection SSD";
}

bool Models::ObjectDetectionSSDModel::enqueue(
  const std::shared_ptr<Engines::Engine> & engine,
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  if (!this->matToBlob(frame, input_frame_loc, 1, 0, engine)) {
    return false;
  }

  setFrameSize(frame.cols, frame.rows);
  return true;
}

bool Models::ObjectDetectionSSDModel::matToBlob(
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

  InferenceEngine::SizeVector blob_size = input_blob->getTensorDesc().getDims();
  const int width = blob_size[3];
  const int height = blob_size[2];
  const int channels = blob_size[1];
  u_int8_t * blob_data = input_blob->buffer().as<u_int8_t *>();

  cv::Mat resized_image(orig_image);
  if (width != orig_image.size().width || height != orig_image.size().height) {
    cv::resize(orig_image, resized_image, cv::Size(width, height));
  }
  int batchOffset = batch_index * width * height * channels;

  for (int c = 0; c < channels; c++) {
    for (int h = 0; h < height; h++) {
      for (int w = 0; w < width; w++) {
        blob_data[batchOffset + c * width * height + h * width + w] =
          resized_image.at<cv::Vec3b>(h, w)[c] * scale_factor;
      }
    }
  }

  return true;
}

bool Models::ObjectDetectionSSDModel::fetchResults(
  const std::shared_ptr<Engines::Engine> & engine,
  std::vector<dynamic_vino_lib::ObjectDetectionResult> & results,
  const float & confidence_thresh,
  const bool & enable_roi_constraint)
{
  if (engine == nullptr) {
    slog::err << "Trying to fetch results from <null> Engines." << slog::endl;
    return false;
  }

  InferenceEngine::InferRequest::Ptr request = engine->getRequest();
  std::string output = getOutputName();
  const float * detections = request->GetBlob(output)->buffer().as<float *>();

  auto max_proposal_count = getMaxProposalCount();
  auto object_size = getObjectSize();
  for (int i = 0; i < max_proposal_count; i++) {
    float image_id = detections[i * object_size + 0];
    if (image_id < 0) {
      // slog::info << "Found objects: " << i << "|" << results.size() << slog::endl;
      break;
    }

    cv::Rect r;
    auto label_num = static_cast<int>(detections[i * object_size + 1]);
    std::vector<std::string> & labels = getLabels();
    auto frame_size = getFrameSize();
    r.x = static_cast<int>(detections[i * object_size + 3] * frame_size.width);
    r.y = static_cast<int>(detections[i * object_size + 4] * frame_size.height);
    r.width = static_cast<int>(detections[i * object_size + 5] * frame_size.width - r.x);
    r.height = static_cast<int>(detections[i * object_size + 6] * frame_size.height - r.y);

    if (enable_roi_constraint) {r &= cv::Rect(0, 0, frame_size.width, frame_size.height);}

    dynamic_vino_lib::ObjectDetectionResult result(r);
    std::string label = label_num < labels.size() ? labels[label_num] :
      std::string("label #") + std::to_string(label_num);
    result.setLabel(label);
    float confidence = detections[i * object_size + 2];
    if (confidence <= confidence_thresh /* || r.x == 0 */) {   // why r.x needs to be checked?
      continue;
    }
    result.setConfidence(confidence);

    results.emplace_back(result);
  }

  return true;
}
