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
 * @brief a header file with declaration of ObjectDetection class and
 * ObjectDetectionResult class
 * @file object_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <stack>
#include "openvino_wrapper_lib/inferences/object_detection.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// ObjectDetectionResult
openvino_wrapper_lib::ObjectDetectionResult::ObjectDetectionResult(const cv::Rect & location)
: Result(location)
{
}

// ObjectDetection
openvino_wrapper_lib::ObjectDetection::ObjectDetection(
  bool enable_roi_constraint,
  double show_output_thresh)
: show_output_thresh_(show_output_thresh),
  enable_roi_constraint_(enable_roi_constraint), openvino_wrapper_lib::BaseInference()
{
  result_filter_ = std::make_shared<Filter>();
  result_filter_->init();
}

openvino_wrapper_lib::ObjectDetection::~ObjectDetection() = default;

void openvino_wrapper_lib::ObjectDetection::loadNetwork(
  std::shared_ptr<Models::ObjectDetectionModel> network)
{
  valid_model_ = network;

  setMaxBatchSize(network->getMaxBatchSize());
}
bool openvino_wrapper_lib::ObjectDetection::enqueue(
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  if (valid_model_ == nullptr || getEngine() == nullptr) {
    return false;
  }

  if (enqueued_frames_ >= valid_model_->getMaxBatchSize()) {
    slog::warn << "Number of " << getName() << "input more than maximum(" <<
      max_batch_size_ << ") processed by inference" << slog::endl;
    return false;
  }

  if (!valid_model_->enqueue(getEngine(), frame, input_frame_loc)) {
    return false;
  }

  enqueued_frames_ += 1;
  return true;
}

bool openvino_wrapper_lib::ObjectDetection::fetchResults()
{
  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch) {
    return false;
  }

  results_.clear();

  return (valid_model_ != nullptr) && valid_model_->fetchResults(
    getEngine(), results_, show_output_thresh_, enable_roi_constraint_);
}

int openvino_wrapper_lib::ObjectDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::ObjectDetection::Result *
openvino_wrapper_lib::ObjectDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::ObjectDetection::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::ObjectDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::ObjectDetection::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!result_filter_->isValidFilterConditions(filter_conditions)) {
    std::vector<cv::Rect> filtered_rois;
    for (auto result : results_) {
      filtered_rois.push_back(result.getLocation());
    }
    return filtered_rois;
  }
  result_filter_->acceptResults(results_);
  result_filter_->acceptFilterConditions(filter_conditions);
  return result_filter_->getFilteredLocations();
}


// ObjectDetectionResultFilter
openvino_wrapper_lib::ObjectDetectionResultFilter::ObjectDetectionResultFilter() {}

void openvino_wrapper_lib::ObjectDetectionResultFilter::init()
{
  key_to_function_.insert(std::make_pair("label", isValidLabel));
  key_to_function_.insert(std::make_pair("confidence", isValidConfidence));
}

void openvino_wrapper_lib::ObjectDetectionResultFilter::acceptResults(
  const std::vector<Result> & results)
{
  results_ = results;
}

std::vector<cv::Rect>
openvino_wrapper_lib::ObjectDetectionResultFilter::getFilteredLocations()
{
  std::vector<cv::Rect> locations;
  for (auto result : results_) {
    if (isValidResult(result)) {
      locations.push_back(result.getLocation());
    }
  }
  return locations;
}

bool openvino_wrapper_lib::ObjectDetectionResultFilter::isValidLabel(
  const Result & result, const std::string & op, const std::string & target)
{
  return stringCompare(result.getLabel(), op, target);
}

bool openvino_wrapper_lib::ObjectDetectionResultFilter::isValidConfidence(
  const Result & result, const std::string & op, const std::string & target)
{
  return floatCompare(result.getConfidence(), op, stringToFloat(target));
}

bool openvino_wrapper_lib::ObjectDetectionResultFilter::isValidResult(
  const Result & result)
{
  ISVALIDRESULT(key_to_function_, result);
}

double openvino_wrapper_lib::ObjectDetection::calcIoU(
  const cv::Rect & box_1,
  const cv::Rect & box_2)
{
  cv::Rect i = box_1 & box_2;
  cv::Rect u = box_1 | box_2;

  return static_cast<double>(i.area()) / static_cast<double>(u.area());
}
