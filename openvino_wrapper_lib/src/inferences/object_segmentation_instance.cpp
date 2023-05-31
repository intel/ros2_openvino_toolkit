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
 
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <openvino/openvino.hpp>
#include "openvino_wrapper_lib/inferences/object_segmentation_instance.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

openvino_wrapper_lib::ObjectSegmentationInstanceResult::ObjectSegmentationInstanceResult(const cv::Rect &location)
    : Result(location)
{
}

openvino_wrapper_lib::ObjectSegmentationInstance::ObjectSegmentationInstance(double show_output_thresh)
    : show_output_thresh_(show_output_thresh), openvino_wrapper_lib::BaseInference()
{
}

openvino_wrapper_lib::ObjectSegmentationInstance::~ObjectSegmentationInstance() = default;

void openvino_wrapper_lib::ObjectSegmentationInstance::loadNetwork(
    const std::shared_ptr<Models::ObjectSegmentationInstanceModel> network)
{
  slog::info << "Loading Network: " << network->getModelCategory() << slog::endl;
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool openvino_wrapper_lib::ObjectSegmentationInstance::enqueue(
    const cv::Mat &frame,
    const cv::Rect &input_frame_loc)
{
  if (width_ == 0 && height_ == 0)
  {
    width_ = frame.cols;
    height_ = frame.rows;
  }

  if (valid_model_ == nullptr || getEngine() == nullptr)
  {
    throw std::logic_error("Model or Engine is not set correctly!");
    return false;
  }

  if (enqueued_frames_ >= valid_model_->getMaxBatchSize())
  {
    slog::warn << "Number of " << getName() << "input more than maximum(" <<
      max_batch_size_ << ") processed by inference" << slog::endl;
    return false;
  }

  if (!valid_model_->enqueue(getEngine(), frame, input_frame_loc))
  {
    return false;
  }

  enqueued_frames_ += 1;
  return true;
}

bool openvino_wrapper_lib::ObjectSegmentationInstance::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::ObjectSegmentationInstance::fetchResults()
{

  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch)
  {
    return false;
  }
  results_.clear();

  return (valid_model_ != nullptr) && valid_model_->fetchResults(
    getEngine(), results_, show_output_thresh_);
}

int openvino_wrapper_lib::ObjectSegmentationInstance::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::ObjectSegmentationInstance::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::ObjectSegmentationInstance::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::ObjectSegmentationInstance::observeOutput(
    const std::shared_ptr<Outputs::BaseOutput> &output)
{
  if (output != nullptr)
  {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::ObjectSegmentationInstance::getFilteredROIs(
    const std::string filter_conditions) const
{
  if (!filter_conditions.empty())
  {
    slog::err << "Object segmentation does not support filtering now! "
              << "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_)
  {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}
