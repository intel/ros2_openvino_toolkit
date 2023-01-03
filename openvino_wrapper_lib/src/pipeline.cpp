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
 * @brief a header file with declaration of Pipeline class
 * @file pipeline.cpp
 */

#include <openvino_param_lib/param_manager.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <openvino/openvino.hpp>

#include "openvino_wrapper_lib/inputs/base_input.hpp"
#include "openvino_wrapper_lib/inputs/image_input.hpp"
#include "openvino_wrapper_lib/pipeline.hpp"

Pipeline::Pipeline(const std::string & name)
{
  if (!name.empty()) {
    params_ = std::make_shared<PipelineParams>(name);
  }
  counter_ = 0;
}

bool Pipeline::add(const std::string & name, std::shared_ptr<Input::BaseInputDevice> input_device)
{
  if (name.empty()) {
    slog::err << "Item name can't be empty!" << slog::endl;
    return false;
  }
  slog::info << "Adding Input Device into Pipeline: " << name << slog::endl;
  input_device_name_ = name;
  input_device_ = input_device;

  addConnect("", name);
  return true;
}

bool Pipeline::add(
  const std::string & parent, const std::string & name,
  std::shared_ptr<Outputs::BaseOutput> output)
{
  if (parent.empty() || name.empty() || !isLegalConnect(parent, name) || output == nullptr) {
    slog::err << "ARGuments ERROR when adding output instance!" << slog::endl;
    return false;
  }

  if (add(name, output)) {
    addConnect(parent, name);

    return true;
  }

  return false;
}

bool Pipeline::add(const std::string & parent, const std::string & name)
{
  if (isLegalConnect(parent, name)) {
    addConnect(parent, name);
    return true;
  }

  return false;
}

bool Pipeline::add(const std::string & name, std::shared_ptr<Outputs::BaseOutput> output)
{
  if (name.empty()) {
    slog::err << "Item name can't be empty!" << slog::endl;
    return false;
  }

  std::map<std::string, std::shared_ptr<Outputs::BaseOutput>>::iterator it =
    name_to_output_map_.find(name);
  if (it != name_to_output_map_.end()) {
    slog::warn << "inferance instance for [" << name <<
      "] already exists, update it with new instance." << slog::endl;
  }
  name_to_output_map_[name] = output;
  output_names_.insert(name);
  /**< Add pipeline instance to Output instance >**/
  output->setPipeline(this);

  return true;
}

void Pipeline::addConnect(const std::string & parent, const std::string & name)
{
  std::pair<std::multimap<std::string, std::string>::iterator,
    std::multimap<std::string, std::string>::iterator>
  ret;
  ret = next_.equal_range(parent);

  for (std::multimap<std::string, std::string>::iterator it = ret.first; it != ret.second; ++it) {
    if (it->second == name) {
      slog::warn << "The connect [" << parent << "<-->" << name << "] already exists." <<
        slog::endl;
      return;
    }
  }
  slog::info << "Adding connection into pipeline:[" << parent << "<-->" << name << "]" <<
    slog::endl;
  next_.insert({parent, name});
}

bool Pipeline::add(
  const std::string & parent, const std::string & name,
  std::shared_ptr<openvino_wrapper_lib::BaseInference> inference)
{
  if (parent.empty() || name.empty() || !isLegalConnect(parent, name)) {
    slog::err << "ARGuments ERROR when adding inference instance!" << slog::endl;
    return false;
  }

  if (add(name, inference)) {
    addConnect(parent, name);
    return true;
  }

  return false;
}

bool Pipeline::add(
  const std::string & name,
  std::shared_ptr<openvino_wrapper_lib::BaseInference> inference)
{
  if (name.empty()) {
    slog::err << "Item name can't be empty!" << slog::endl;
    return false;
  }

  std::map<std::string, std::shared_ptr<openvino_wrapper_lib::BaseInference>>::iterator it =
    name_to_detection_map_.find(name);
  if (it != name_to_detection_map_.end()) {
    slog::warn << "inferance instance for [" << name <<
      "] already exists, update it with new instance." << slog::endl;
  } else {
    ++total_inference_;
  }
  name_to_detection_map_[name] = inference;

  return true;
}

bool Pipeline::isLegalConnect(const std::string parent, const std::string child)
{
  int parent_order = getCatagoryOrder(parent);
  int child_order = getCatagoryOrder(child);
  slog::info << "Checking connection into pipeline:[" << parent << "(" << parent_order << ")" <<
    "<-->" << child << "(" << child_order << ")" <<
    "]" << slog::endl;
  return (parent_order != kCatagoryOrder_Unknown) && (child_order != kCatagoryOrder_Unknown) &&
         (parent_order <= child_order);
}

int Pipeline::getCatagoryOrder(const std::string name)
{
  int order = kCatagoryOrder_Unknown;
  if (name == input_device_name_) {
    order = kCatagoryOrder_Input;
  } else if (name_to_detection_map_.find(name) != name_to_detection_map_.end()) {
    order = kCatagoryOrder_Inference;
  } else if (name_to_output_map_.find(name) != name_to_output_map_.end()) {
    order = kCatagoryOrder_Output;
  }

  return order;
}

void Pipeline::runOnce()
{
  initInferenceCounter();

  if (!input_device_->read(&frame_)) {
    return; //do nothing if now frame read out
  }
  width_ = frame_.cols;
  height_ = frame_.rows;
  slog::debug << "DEBUG: in Pipeline run process..." << slog::endl;

  for (auto pos = next_.equal_range(input_device_name_); pos.first != pos.second; ++pos.first) {
    std::string detection_name = pos.first->second;
    slog::debug << "DEBUG: Enqueue for detection: " << detection_name << slog::endl;
    auto detection_ptr = name_to_detection_map_[detection_name];
    detection_ptr->enqueue(frame_, cv::Rect(width_ / 2, height_ / 2, width_, height_));
    increaseInferenceCounter();
    slog::debug << "DEBUG: Submit Infer request for detection: " << detection_name << slog::endl;
    detection_ptr->submitRequest();
  }

  for (auto &pair : name_to_output_map_)
  {
    pair.second->feedFrame(frame_);
  }
  countFPS();

  slog::debug << "DEBUG: align inference process, waiting until all inferences done!" << slog::endl;
  std::unique_lock<std::mutex> lock(counter_mutex_);
  cv_.wait(lock, [self = this]() {return self->counter_ == 0;});

  slog::debug << "DEBUG: in Pipeline run process...handleOutput" << slog::endl;
  for (auto & pair : name_to_output_map_) {
    pair.second->handleOutput();
  }
}

void Pipeline::printPipeline()
{
  for (auto & current_node : next_) {
    printf("%s --> %s\n", current_node.first.c_str(), current_node.second.c_str());
  }
}

void Pipeline::setCallback()
{
  for (auto & pair : name_to_detection_map_) {
    std::string detection_name = pair.first;
    std::function<void(std::__exception_ptr::exception_ptr)> callb;
    callb = [detection_name, self = this](std::exception_ptr ex)
      {
        if (ex)
          throw ex;

        self->callback(detection_name);
        return;
      };
    pair.second->getEngine()->getRequest().set_callback(callb);
   }
}

void Pipeline::callback(const std::string & detection_name)
{
  slog::debug <<"Hello callback ----> " << detection_name <<slog::endl;
  auto detection_ptr = name_to_detection_map_[detection_name];
  detection_ptr->fetchResults();
  // set output
  for (auto pos = next_.equal_range(detection_name); pos.first != pos.second; ++pos.first) {
    std::string next_name = pos.first->second;

    std::string filter_conditions = findFilterConditions(detection_name, next_name);

    if (output_names_.find(next_name) != output_names_.end()) {
      detection_ptr->observeOutput(name_to_output_map_[next_name]);
    } else {
      auto detection_ptr_iter = name_to_detection_map_.find(next_name);
      if (detection_ptr_iter != name_to_detection_map_.end()) {
        auto next_detection_ptr = detection_ptr_iter->second;
        size_t batch_size = next_detection_ptr->getMaxBatchSize();
        std::vector<cv::Rect> next_rois = detection_ptr->getFilteredROIs(filter_conditions);
        for (size_t i = 0; i < next_rois.size(); i++) {
          auto roi = next_rois[i];
          auto clippedRect = roi & cv::Rect(0, 0, width_, height_);
          cv::Mat next_input = frame_(clippedRect);
          next_detection_ptr->enqueue(next_input, roi);
          if ((i + 1) == next_rois.size() || (i + 1) % batch_size == 0) {
            increaseInferenceCounter();
            next_detection_ptr->submitRequest();
            auto request = next_detection_ptr->getEngine()->getRequest();
            request.wait();
          }
        }
      }
    }
  }

  decreaseInferenceCounter();
  cv_.notify_all();
}

void Pipeline::initInferenceCounter()
{
  std::lock_guard<std::mutex> lk(counter_mutex_);
  counter_ = 0;
  cv_.notify_all();
}

void Pipeline::increaseInferenceCounter()
{
  std::lock_guard<std::mutex> lk(counter_mutex_);
  ++counter_;
}

void Pipeline::decreaseInferenceCounter()
{
  std::lock_guard<std::mutex> lk(counter_mutex_);
  --counter_;
}

void Pipeline::countFPS()
{
  frame_cnt_++;
  auto t_end = std::chrono::high_resolution_clock::now();
  typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
  ms secondDetection = std::chrono::duration_cast<ms>(t_end - t_start_);
  if (secondDetection.count() > 1000) {  
    setFPS(frame_cnt_);
    frame_cnt_ = 0;
    t_start_ = t_end;
  }
}
