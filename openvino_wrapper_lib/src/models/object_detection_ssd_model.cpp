// Copyright (c) 2018-2022 Intel Corporation
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
#include <algorithm>
#include <string>
#include <memory>
#include <unordered_set>
#include <vector>
#include <openvino/op/constant.hpp>
#include <openvino/op/convert.hpp>
#include "openvino_wrapper_lib/inferences/object_detection.hpp"
#include "openvino_wrapper_lib/models/object_detection_ssd_model.hpp"
#include "openvino_wrapper_lib/slog.hpp"
#include "openvino_wrapper_lib/engines/engine.hpp"
#include "openvino_wrapper_lib/models/attributes/base_attribute.hpp"

namespace
{
// TensorFlow tensor names may carry an output index suffix, e.g. "detection_boxes:0".
std::string findOutputName(const std::shared_ptr<ov::Model>& model, const std::string& name)
{
  for (const auto& output : model->outputs()) {
    for (const auto& tensor_name : output.get_names()) {
      if (tensor_name == name || tensor_name.rfind(name + ":", 0) == 0) {
        return tensor_name;
      }
    }
  }
  return "";
}

using NodeSet = std::unordered_set<const ov::Node*>;

NodeSet collectNodes(const std::vector<std::shared_ptr<ov::Node>>& roots, bool downstream)
{
  NodeSet seen;
  std::vector<ov::Node*> stack;
  for (const auto& root : roots) {
    stack.push_back(root.get());
  }
  while (!stack.empty()) {
    ov::Node* node = stack.back();
    stack.pop_back();
    if (!seen.insert(node).second) {
      continue;
    }
    if (downstream) {
      for (const auto& output : node->outputs()) {
        for (const auto& target : output.get_target_inputs()) {
          stack.push_back(target.get_node());
        }
      }
    } else {
      for (const auto& input : node->inputs()) {
        stack.push_back(input.get_source_output().get_node());
      }
    }
  }
  return seen;
}

// The box encodings and class predictions that the TensorFlow post-processing reads.
std::vector<std::shared_ptr<ov::Node>> findPredictionHeads(const std::shared_ptr<ov::Model>& model)
{
  std::vector<std::shared_ptr<ov::Node>> heads;
  for (const auto& op : model->get_ordered_ops()) {
    if (op->get_friendly_name() == "concat" || op->get_friendly_name() == "concat_1") {
      heads.push_back(op);
    }
  }
  return heads;
}

// The TensorFlow pre-processor resizes each image in a Loop. With a 300x300 input that
// resize is an identity, and removing the Loop lets the backbone run on the NPU.
bool removeIdentityResizeLoop(const std::shared_ptr<ov::Model>& model)
{
  const auto heads = findPredictionHeads(model);
  if (heads.size() != 2) {
    return false;
  }
  const auto upstream = collectNodes(heads, false);
  for (const auto& op : model->get_ordered_ops()) {
    if (std::string(op->get_type_name()) != "Loop" || upstream.count(op.get()) == 0) {
      continue;
    }
    if (op->get_output_size() != 2 || op->get_output_partial_shape(0) != ov::PartialShape{ 1, 300, 300, 3 } ||
        op->get_output_partial_shape(1) != ov::PartialShape{ 1, 3 }) {
      return false;
    }
    auto image = std::make_shared<ov::op::v0::Convert>(model->get_parameters()[0], op->get_output_element_type(0));
    auto image_shape = ov::op::v0::Constant::create(op->get_output_element_type(1), ov::Shape{ 1, 3 }, { 300, 300, 3 });
    op->output(0).replace(image->output(0));
    op->output(1).replace(image_shape->output(0));
    model->validate_nodes_and_infer_types();
    return true;
  }
  return false;
}
}  // namespace

// Validated Object Detection Network
Models::ObjectDetectionSSDModel::ObjectDetectionSSDModel(const std::string& label_loc, const std::string& model_loc,
                                                         int max_batch_size)
  : ObjectDetectionModel(label_loc, model_loc, max_batch_size)
{
  slog::debug << "TESTING: in ObjectDetectionSSDModel" << slog::endl;
}

const std::string Models::ObjectDetectionSSDModel::getModelCategory() const
{
  return "Object Detection SSD";
}

bool Models::ObjectDetectionSSDModel::enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
                                              const cv::Rect& input_frame_loc)
{
  if (!this->matToBlob(frame, input_frame_loc, 1, 0, engine)) {
    return false;
  }

  setFrameSize(frame.cols, frame.rows);
  return true;
}

bool Models::ObjectDetectionSSDModel::matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor,
                                                int batch_index, const std::shared_ptr<Engines::Engine>& engine)
{
  if (engine == nullptr) {
    slog::err << "A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  std::string input_name = getInputName();
  slog::debug << "add input image to blob: " << input_name << slog::endl;

  ov::Tensor in_tensor = engine->getRequest().get_tensor(input_name);

  ov::Shape in_shape = in_tensor.get_shape();
  const int width = in_shape[3];
  const int height = in_shape[2];
  const int channels = in_shape[1];

  u_int8_t* blob_data = (u_int8_t*)in_tensor.data();

  cv::Mat resized_image(orig_image);
  if (width != orig_image.size().width || height != orig_image.size().height) {
    cv::resize(orig_image, resized_image, cv::Size(width, height));
  }
  int batchOffset = batch_index * width * height * channels;

  for (int c = 0; c < channels; c++) {
    const int src_c = tf_api_outputs_ ? channels - 1 - c : c;
    for (int h = 0; h < height; h++) {
      for (int w = 0; w < width; w++) {
        blob_data[batchOffset + c * width * height + h * width + w] =
            resized_image.at<cv::Vec3b>(h, w)[src_c] * scale_factor;
      }
    }
  }

  slog::debug << "Convert input image to blob: DONE!" << slog::endl;
  return true;
}

bool Models::ObjectDetectionSSDModel::fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                                                   std::vector<openvino_wrapper_lib::ObjectDetectionResult>& results,
                                                   const float& confidence_thresh, const bool& enable_roi_constraint)
{
  slog::debug << "fetching Infer Results from the given SSD model" << slog::endl;
  if (engine == nullptr) {
    slog::err << "Trying to fetch results from <null> Engines." << slog::endl;
    return false;
  }

  slog::debug << "Fetching Detection Results ..." << slog::endl;
  ov::InferRequest request = engine->getRequest();
  if (tf_api_outputs_) {
    return fetchTfApiResults(request, results, confidence_thresh, enable_roi_constraint);
  }
  std::string output = getOutputName();
  const float* detections = (float*)request.get_tensor(output).data();

  slog::debug << "Analyzing Detection results..." << slog::endl;
  auto max_proposal_count = getMaxProposalCount();
  auto object_size = getObjectSize();
  slog::debug << "MaxProposalCount=" << max_proposal_count << ", ObjectSize=" << object_size << slog::endl;
  for (int i = 0; i < max_proposal_count; i++) {
    float image_id = detections[i * object_size + 0];
    if (image_id < 0) {
      break;
    }

    cv::Rect r;
    auto label_num = static_cast<int>(detections[i * object_size + 1]);
    auto frame_size = getFrameSize();
    r.x = static_cast<int>(detections[i * object_size + 3] * frame_size.width);
    r.y = static_cast<int>(detections[i * object_size + 4] * frame_size.height);
    r.width = static_cast<int>(detections[i * object_size + 5] * frame_size.width - r.x);
    r.height = static_cast<int>(detections[i * object_size + 6] * frame_size.height - r.y);

    if (enable_roi_constraint) {
      r &= cv::Rect(0, 0, frame_size.width, frame_size.height);
    }

    openvino_wrapper_lib::ObjectDetectionResult result(r);
    float confidence = detections[i * object_size + 2];
    if (confidence <= confidence_thresh) {
      continue;
    }
    result.setLabel(getLabel(label_num));
    result.setConfidence(confidence);

    results.emplace_back(result);
  }

  return true;
}

bool Models::ObjectDetectionSSDModel::fetchTfApiResults(
    ov::InferRequest& request, std::vector<openvino_wrapper_lib::ObjectDetectionResult>& results,
    float confidence_thresh, bool enable_roi_constraint)
{
  const ov::Tensor boxes = request.get_tensor(boxes_name_);
  const ov::Tensor scores = request.get_tensor(scores_name_);
  const ov::Tensor classes = request.get_tensor(classes_name_);
  const float* box_data = boxes.data<float>();
  const float* score_data = scores.data<float>();
  const float* class_data = classes.data<float>();

  size_t count = scores.get_size();
  if (!num_detections_name_.empty()) {
    const ov::Tensor num = request.get_tensor(num_detections_name_);
    count = std::min(count, static_cast<size_t>(num.data<float>()[0]));
  }

  const auto frame_size = getFrameSize();
  for (size_t i = 0; i < count; i++) {
    const float confidence = score_data[i];
    if (confidence <= confidence_thresh) {
      continue;
    }
    // Boxes are normalized [y_min, x_min, y_max, x_max].
    const float* box = box_data + i * 4;
    cv::Rect r;
    r.x = static_cast<int>(box[1] * frame_size.width);
    r.y = static_cast<int>(box[0] * frame_size.height);
    r.width = static_cast<int>(box[3] * frame_size.width - r.x);
    r.height = static_cast<int>(box[2] * frame_size.height - r.y);
    if (enable_roi_constraint) {
      r &= cv::Rect(0, 0, frame_size.width, frame_size.height);
    }

    openvino_wrapper_lib::ObjectDetectionResult result(r);
    result.setLabel(getLabel(static_cast<int>(class_data[i])));
    result.setConfidence(confidence);
    results.emplace_back(result);
  }
  return true;
}

bool Models::ObjectDetectionSSDModel::updateLayerProperty(std::shared_ptr<ov::Model>& model)
{
  slog::info << "Checking INPUTs for model " << getModelName() << slog::endl;

  auto input_info_map = model->inputs();
  if (input_info_map.size() != 1) {
    slog::warn << "This model seems not SSDNet-like, SSDnet has only one input, but we got "
               << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    return false;
  }

  boxes_name_ = findOutputName(model, "detection_boxes");
  scores_name_ = findOutputName(model, "detection_scores");
  classes_name_ = findOutputName(model, "detection_classes");
  num_detections_name_ = findOutputName(model, "num_detections");
  tf_api_outputs_ = !boxes_name_.empty() && !scores_name_.empty() && !classes_name_.empty();
  if (tf_api_outputs_ && model->input().get_partial_shape().is_dynamic()) {
    // The graph resizes to its own input size internally; 300x300 is the SSD training size.
    slog::info << "TensorFlow Object Detection API model, fixing input shape to [1,300,300,3]" << slog::endl;
    model->reshape(ov::PartialShape{ 1, 300, 300, 3 });
    input_info_map = model->inputs();
  }
  if (tf_api_outputs_) {
    resize_loop_removed_ = removeIdentityResizeLoop(model);
  }

  ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
  input_tensor_name_ = model->input().get_any_name();
  ov::preprocess::InputInfo& input_info = ppp.input(input_tensor_name_);

  input_info.tensor().set_element_type(ov::element::u8);
  addInputInfo(ModelAttribute::DefaultInputName, input_tensor_name_);

  ov::Shape input_dims = input_info_map[0].get_shape();

  ov::Layout tensor_layout = ov::Layout("NCHW");
  ov::Layout expect_layout = ov::Layout("NHWC");
  setInputHeight(input_dims[2]);
  setInputWidth(input_dims[3]);
  if (input_dims[1] == 3)
    expect_layout = ov::Layout("NCHW");
  else if (input_dims[3] == 3) {
    expect_layout = ov::Layout("NHWC");
    setInputHeight(input_dims[1]);
    setInputWidth(input_dims[2]);
  } else
    slog::warn << "unexpected input shape " << input_dims << slog::endl;

  input_info.tensor().set_element_type(ov::element::u8).set_layout(tensor_layout);
  input_info.preprocess().convert_layout(expect_layout).resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR);

  slog::info << "Checking OUTPUTs for model " << getModelName() << slog::endl;
  if (tf_api_outputs_) {
    for (const auto& name : { boxes_name_, scores_name_, classes_name_, num_detections_name_ }) {
      if (!name.empty()) {
        ppp.output(name).tensor().set_element_type(ov::element::f32);
      }
    }
    addOutputInfo(ModelAttribute::DefaultOutputName, scores_name_);
    model = ppp.build();
    printAttribute();
    slog::info << "TensorFlow Object Detection API outputs found, Layer Property updated!" << slog::endl;
    return true;
  }

  auto outputs_info = model->outputs();
  if (outputs_info.size() != 1) {
    slog::warn << "This model seems not SSDNet-like! We got " << std::to_string(outputs_info.size())
               << "outputs, but SSDnet has only one." << slog::endl;
    return false;
  }
  ov::preprocess::OutputInfo& output_info = ppp.output();
  addOutputInfo(ModelAttribute::DefaultOutputName, model->output().get_any_name());
  slog::info << "Checking Object Detection output ... Name=" << model->output().get_any_name() << slog::endl;

  output_info.tensor().set_element_type(ov::element::f32);
  model = ppp.build();

  // last dimension of output layer should be 7
  auto outputsDataMap = model->outputs();
  auto& data = outputsDataMap[0];
  ov::Shape output_dims = data.get_shape();
  setMaxProposalCount(static_cast<int>(output_dims[2]));
  slog::info << "max proposal count is: " << getMaxProposalCount() << slog::endl;

  auto object_size = static_cast<int>(output_dims[3]);
  if (object_size != 7) {
    slog::warn << "This model is NOT SSDNet-like, whose output data for each detected object"
               << "should have 7 dimensions, but was " << std::to_string(object_size) << slog::endl;
    return false;
  }
  setObjectSize(object_size);

  if (output_dims.size() != 4) {
    slog::warn << "This model is not SSDNet-like, output dimensions should be 4, but was"
               << std::to_string(output_dims.size()) << slog::endl;
    return false;
  }

  printAttribute();
  slog::info << "This model is SSDNet-like, Layer Property updated!" << slog::endl;
  return true;
}

std::string Models::ObjectDetectionSSDModel::prepareForDevice(const std::string& device)
{
  // The NPU cannot compile the dynamic shapes of the TensorFlow post-processing (NMS),
  // so only the backbone runs on the NPU and the post-processing on the CPU.
  if (!tf_api_outputs_ || device.rfind("NPU", 0) != 0 || device.find(':') != std::string::npos) {
    return device;
  }
  if (!resize_loop_removed_) {
    slog::warn << "The TensorFlow pre-processing of this model cannot run on " << device << ", using CPU"
               << slog::endl;
    return "CPU";
  }

  const auto heads = findPredictionHeads(model_);
  const auto upstream = collectNodes(heads, false);
  const auto from_input = collectNodes({ model_->get_parameters()[0] }, true);
  // The backbone starts where the image becomes floating point. The u8 layout and resize
  // pre-processing before it stays on the CPU: on the NPU it gave no detections.
  std::shared_ptr<ov::Node> image_input;
  for (const auto& op : model_->get_ordered_ops()) {
    if (upstream.count(op.get()) != 0 && from_input.count(op.get()) != 0 &&
        op->get_output_element_type(0).is_real()) {
      image_input = op;
      break;
    }
  }
  NodeSet npu;
  if (image_input != nullptr) {
    const auto downstream = collectNodes({ image_input }, true);
    for (const auto* node : upstream) {
      if (downstream.count(node) != 0) {
        npu.insert(node);
      }
    }
  }
  // Keep shape and index computations of the CPU part off the NPU; only image tensors cross.
  for (bool changed = true; changed;) {
    changed = false;
    for (const auto& op : model_->get_ordered_ops()) {
      if (npu.count(op.get()) == 0) {
        continue;
      }
      for (const auto& input : op->inputs()) {
        const auto* source = input.get_source_output().get_node();
        if (npu.count(source) == 0 && !ov::is_type<ov::op::v0::Constant>(source) &&
            input.get_partial_shape().rank() != 4) {
          npu.erase(op.get());
          changed = true;
          break;
        }
      }
    }
  }
  if (npu.empty()) {
    slog::warn << "No operation of this model can be assigned to " << device << ", using CPU" << slog::endl;
    return "CPU";
  }
  for (const auto& op : model_->get_ordered_ops()) {
    bool on_npu = npu.count(op.get()) != 0;
    if (ov::is_type<ov::op::v0::Constant>(op)) {
      const auto targets = op->output(0).get_target_inputs();
      on_npu = std::all_of(targets.begin(), targets.end(),
                           [&npu](const ov::Input<ov::Node>& target) { return npu.count(target.get_node()) != 0; });
    }
    op->get_rt_info()["affinity"] = on_npu ? device : std::string("CPU");
  }

  const std::string hetero = "HETERO:" + device + ",CPU";
  slog::info << "Running the backbone (" << npu.size() << " operations) on " << device
             << " and the TensorFlow post-processing on CPU (" << hetero << ")" << slog::endl;
  return hetero;
}
