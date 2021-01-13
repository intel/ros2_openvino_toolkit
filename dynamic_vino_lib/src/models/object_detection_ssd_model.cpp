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
 * @file object_detection_ssd_model.cpp
 */
#include <string>
#include <memory>
#include <vector>
#include "dynamic_vino_lib/inferences/object_detection.hpp"
#include "dynamic_vino_lib/models/object_detection_ssd_model.hpp"
#include "dynamic_vino_lib/slog.hpp"
#include "dynamic_vino_lib/engines/engine.hpp"
#include "dynamic_vino_lib/models/attributes/base_attribute.hpp"

// Validated Object Detection Network
Models::ObjectDetectionSSDModel::ObjectDetectionSSDModel(
  const std::string & model_loc, int max_batch_size)
: ObjectDetectionModel(model_loc, max_batch_size)
{
  slog::debug << "TESTING: in ObjectDetectionSSDModel" << slog::endl;
  //addCandidatedAttr(std::make_shared<Models::SSDModelAttr>());
}

const std::string Models::ObjectDetectionSSDModel::getModelCategory() const
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
  slog::debug << "add input image to blob: " << input_name << slog::endl;
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

  slog::debug << "Convert input image to blob: DONE!" << slog::endl;
  return true;
}

bool Models::ObjectDetectionSSDModel::fetchResults(
  const std::shared_ptr<Engines::Engine> & engine,
  std::vector<dynamic_vino_lib::ObjectDetectionResult> & results,
  const float & confidence_thresh,
  const bool & enable_roi_constraint)
{
  slog::debug << "fetching Infer Resulsts from the given SSD model" << slog::endl;
  if (engine == nullptr) {
    slog::err << "Trying to fetch results from <null> Engines." << slog::endl;
    return false;
  }

  slog::debug << "Fetching Detection Results ..." << slog::endl;
  InferenceEngine::InferRequest::Ptr request = engine->getRequest();
  std::string output = getOutputName();
  const float * detections = request->GetBlob(output)->buffer().as<float *>();

  slog::debug << "Analyzing Detection results..." << slog::endl;
  auto max_proposal_count = getMaxProposalCount();
  auto object_size = getObjectSize();
  slog::debug << "MaxProprosalCount=" << max_proposal_count
    << ", ObjectSize=" << object_size << slog::endl;
  for (int i = 0; i < max_proposal_count; i++) {
    float image_id = detections[i * object_size + 0];
    if (image_id < 0) {
      //slog::info << "Found objects: " << i << "|" << results.size() << slog::endl;
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

bool Models::ObjectDetectionSSDModel::updateLayerProperty(
  const InferenceEngine::CNNNetReader::Ptr net_reader)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;

  InferenceEngine::InputsDataMap input_info_map(net_reader->getNetwork().getInputsInfo());
  if (input_info_map.size() != 1) {
    slog::warn << "This model seems not SSDNet-like, SSDnet has only one input, but we got "
      << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    return false;
  }
  
  InferenceEngine::InputInfo::Ptr input_info = input_info_map.begin()->second;
  input_info->setPrecision(InferenceEngine::Precision::U8);
  addInputInfo("input", input_info_map.begin()->first);

  const InferenceEngine::SizeVector input_dims = input_info->getTensorDesc().getDims();
  setInputHeight(input_dims[2]);
  setInputWidth(input_dims[3]);

  slog::info << "Checking OUTPUTs for model " << getModelName() << slog::endl;
  InferenceEngine::OutputsDataMap output_info_map(net_reader->getNetwork().getOutputsInfo());
  if (output_info_map.size() != 1) {
    slog::warn << "This model seems not SSDNet-like! We got " 
      << std::to_string(output_info_map.size()) << "outputs, but SSDnet has only one."
      << slog::endl;
    return false;
  }
  InferenceEngine::DataPtr & output_data_ptr = output_info_map.begin()->second;
  addOutputInfo("output", output_info_map.begin()->first);
  slog::info << "Checking Object Detection output ... Name=" << output_info_map.begin()->first
    << slog::endl;
  output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);

///TODO: double check this part: BEGIN
  const InferenceEngine::CNNLayerPtr output_layer =
    net_reader->getNetwork().getLayerByName(output_info_map.begin()->first.c_str());
  // output layer should have attribute called num_classes
  slog::info << "Checking Object Detection num_classes" << slog::endl;
  if (output_layer->params.find("num_classes") == output_layer->params.end()) {
    slog::warn << "This model's output layer (" << output_info_map.begin()->first
      << ") should have num_classes integer attribute" << slog::endl;
    return false;
  }
  // class number should be equal to size of label vector
  // if network has default "background" class, fake is used
  const int num_classes = output_layer->GetParamAsInt("num_classes");
#if 0
  slog::info << "Checking Object Detection output ... num_classes=" << num_classes << slog::endl;
  if (getLabels().size() != num_classes) {
    if (getLabels().size() == (num_classes - 1)) {
      getLabels().insert(getLabels().begin(), "fake");
    } else {
      getLabels().clear();
    }
  }
#endif
  ///TODO: double check this part: END

  // last dimension of output layer should be 7
  const InferenceEngine::SizeVector output_dims = output_data_ptr->getTensorDesc().getDims();
  setMaxProposalCount(static_cast<int>(output_dims[2]));
  slog::info << "max proposal count is: " << getMaxProposalCount() << slog::endl;

  auto object_size = static_cast<int>(output_dims[3]);
  if (object_size != 7) {
    slog::warn << "This model is NOT SSDNet-like, whose output data for each detected object"
      << "should have 7 dimensions, but was " << std::to_string(object_size)
      << slog::endl;
    return false;
  }
  setObjectSize(object_size);

  if (output_dims.size() != 4) {
    slog::warn << "This model is not SSDNet-like, output dimensions shoulld be 4, but was"
      << std::to_string(output_dims.size()) << slog::endl;
    return false;
  }

  printAttribute();
  slog::info << "This model is SSDNet-like, Layer Property updated!" << slog::endl;
  return true;
}
