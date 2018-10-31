/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief a header file with declaration of Pipeline class
 * @file pipeline.cpp
 */

#include <utility>
#include <memory>
#include <string>

#include "dynamic_vino_lib/pipeline.hpp"
#include <vino_param_lib/param_manager.hpp>

using namespace InferenceEngine;

Pipeline::Pipeline(const std::string& name) {
  params_ = std::make_shared<PipelineParams>(name);
  counter_ = 0;
}

bool Pipeline::add(const std::string& name,
                   std::shared_ptr<Input::BaseInputDevice> input_device) {
  input_device_name_ = name;
  input_device_ = std::move(input_device);
  next_.insert({"", name});
  return true;
}

bool Pipeline::add(const std::string& parent, const std::string& name,
                   std::shared_ptr<Outputs::BaseOutput> output) {
  if (parent.empty()) {
    slog::err << "output device have no parent!" << slog::endl;
    return false;
  }
  if (name_to_detection_map_.find(parent) == name_to_detection_map_.end()) {
    slog::err << "parent detection does not exists!" << slog::endl;
    return false;
  }
  if (output_names_.find(name) != output_names_.end()) {
    return add(parent, name);
  }
  output_names_.insert(name);
  name_to_output_map_[name] = std::move(output);
  next_.insert({parent, name});
  
  /**< Add pipeline instance to Output instance >**/
  output->setPipeline(this);
  return true;
}

bool Pipeline::add(const std::string& parent, const std::string& name) {
  if (parent.empty()) {
    slog::err << "output device should have no parent!" << slog::endl;
    return false;
  }
  if (name_to_detection_map_.find(parent) == name_to_detection_map_.end()) {
    slog::err << "parent detection does not exists!" << slog::endl;
    return false;
  }
  if (std::find(output_names_.begin(), output_names_.end(), name) ==
      output_names_.end()) {
    slog::err << "output does not exists!" << slog::endl;
    return false;
  }
  next_.insert({parent, name});
  return true;
}

bool Pipeline::add(const std::string& parent, const std::string& name,
                   std::shared_ptr<dynamic_vino_lib::BaseInference> inference) {
  if (name_to_detection_map_.find(parent) == name_to_detection_map_.end() &&
      input_device_name_ != parent) {
    slog::err << "parent device/detection does not exists!" << slog::endl;
    return false;
  }
  next_.insert({parent, name});
  name_to_detection_map_[name] = std::move(inference);
  ++total_inference_;
  return true;
}

void Pipeline::runOnce(const std::string& input_type) {
  initInferenceCounter();

  if (!input_device_->read(&frame_)) {
    //throw std::logic_error("Failed to get frame from cv::VideoCapture");
    slog::warn << "Failed to get frame from input_device." << slog::endl;
    return;
  }
  width_ = frame_.cols;
  height_ = frame_.rows;
  for (auto& pair : name_to_output_map_) {
    pair.second->feedFrame(frame_);
  }
  auto t0 = std::chrono::high_resolution_clock::now();
  for (auto pos = next_.equal_range(input_device_name_);
       pos.first != pos.second; ++pos.first) {
    std::string detection_name = pos.first->second;
    auto detection_ptr = name_to_detection_map_[detection_name];
    detection_ptr->enqueue(frame_,
                           cv::Rect(width_ / 2, height_ / 2, width_, height_));
    increaseInferenceCounter();
    detection_ptr->submitRequest();
  }
  std::unique_lock<std::mutex> lock(counter_mutex_);
  cv_.wait(lock, [self = this]() { return self->counter_ == 0; });
  auto t1 = std::chrono::high_resolution_clock::now();
  typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

  for (auto& pair : name_to_output_map_) {
    pair.second->handleOutput();
  }
}

void Pipeline::printPipeline() {
  for (auto& current_node : next_) {
    printf("%s --> %s\n", current_node.first.c_str(),
           current_node.second.c_str());
  }
}

void Pipeline::setCallback() {
#if 0
  if (!input_device_->read(&frame_)) {
    throw std::logic_error("Failed to get frame from cv::VideoCapture");
  }
  width_ = frame_.cols;
  height_ = frame_.rows;
  for (auto& pair : name_to_output_map_) {
    pair.second->feedFrame(frame_);
  }
#endif
  for (auto& pair : name_to_detection_map_) {
    std::string detection_name = pair.first;
    std::function<void(void)> callb;
    callb = [ detection_name, self = this ]() {
      self->callback(detection_name);
      return;
    };
    pair.second->getEngine()->getRequest()->SetCompletionCallback(callb);
  }
}
void Pipeline::callback(const std::string& detection_name) {
   // slog::info<<"Hello callback ----> " << detection_name <<slog::endl;
  auto detection_ptr = name_to_detection_map_[detection_name];
  detection_ptr->fetchResults();
  // set output
  for (auto pos = next_.equal_range(detection_name); pos.first != pos.second;
       ++pos.first) {
    std::string next_name = pos.first->second;
    // if next is output, then print
    if (output_names_.find(next_name) != output_names_.end()) {
      // name_to_output_map_[next_name]->accept(*detection_ptr->getResult());
      detection_ptr->observeOutput(name_to_output_map_[next_name]);
    }else {
      // slog::info << "Inference ... ";
      auto detection_ptr_iter = name_to_detection_map_.find(next_name);
      if (detection_ptr_iter != name_to_detection_map_.end()) {
        auto next_detection_ptr = detection_ptr_iter->second;
        for (size_t i = 0; i < detection_ptr->getResultsLength(); ++i) {
          const dynamic_vino_lib::Result* prev_result =
              detection_ptr->getLocationResult(i);
          auto clippedRect =
              prev_result->getLocation() & cv::Rect(0, 0, width_, height_);
          cv::Mat next_input = frame_(clippedRect);
          next_detection_ptr->enqueue(next_input, prev_result->getLocation());
        }
        if (detection_ptr->getResultsLength() > 0) {
          increaseInferenceCounter();
          next_detection_ptr->submitRequest();
        }
      }
    }
  }

  decreaseInferenceCounter();
  cv_.notify_all();
}

void Pipeline::initInferenceCounter() {
  std::lock_guard<std::mutex> lk(counter_mutex_);
  counter_ = 0;
  cv_.notify_all();
}
void Pipeline::increaseInferenceCounter() {
  std::lock_guard<std::mutex> lk(counter_mutex_);
  ++counter_;
  // slog::info << "counter = " << counter_ << slog::endl;
}
void Pipeline::decreaseInferenceCounter() {
  std::lock_guard<std::mutex> lk(counter_mutex_);
  --counter_;
  // slog::info << "counter = " << counter_ << slog::endl;
}
