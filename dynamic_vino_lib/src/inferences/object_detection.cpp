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
 * @brief a header file with declaration of ObjectDetection class and
 * ObjectDetectionResult class
 * @file object_detection.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <stack>
#include "dynamic_vino_lib/inferences/object_detection.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"
// ObjectDetectionResult
dynamic_vino_lib::ObjectDetectionResult::ObjectDetectionResult(const cv::Rect & location)
: Result(location)
{
}
// ObjectDetection
dynamic_vino_lib::ObjectDetection::ObjectDetection(
  bool enable_roi_constraint,
  double show_output_thresh)
: show_output_thresh_(show_output_thresh),
  enable_roi_constraint_(enable_roi_constraint), dynamic_vino_lib::BaseInference()
{
  result_filter_ = std::make_shared<Filter>();
  result_filter_->init();
}

dynamic_vino_lib::ObjectDetection::~ObjectDetection() = default;
void dynamic_vino_lib::ObjectDetection::loadNetwork(
  const std::shared_ptr<Models::ObjectDetectionModel> network)
{
  valid_model_ = network;
  max_proposal_count_ = network->getMaxProposalCount();
  object_size_ = network->getObjectSize();
  setMaxBatchSize(network->getMaxBatchSize());
}
bool dynamic_vino_lib::ObjectDetection::enqueue(
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
  width_ = frame.cols;
  height_ = frame.rows;
  if (!dynamic_vino_lib::BaseInference::enqueue<u_int8_t>(frame, input_frame_loc, 1, 0,
    valid_model_->getInputName()))
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.clear();
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::ObjectDetection::submitRequest()
{
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::ObjectDetection::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) {
    return false;
  }
  bool found_result = false;
  results_.clear();
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float * detections = request->GetBlob(output)->buffer().as<float *>();
  for (int i = 0; i < max_proposal_count_; i++) {
    float image_id = detections[i * object_size_ + 0];
    cv::Rect r;
    auto label_num = static_cast<int>(detections[i * object_size_ + 1]);
    std::vector<std::string> & labels = valid_model_->getLabels();
    r.x = static_cast<int>(detections[i * object_size_ + 3] * width_);
    r.y = static_cast<int>(detections[i * object_size_ + 4] * height_);
    r.width = static_cast<int>(detections[i * object_size_ + 5] * width_ - r.x);
    r.height = static_cast<int>(detections[i * object_size_ + 6] * height_ - r.y);
    if (enable_roi_constraint_) {r &= cv::Rect(0, 0, width_, height_);}
    Result result(r);
    result.label_ = label_num < labels.size() ? labels[label_num] :
      std::string("label #") + std::to_string(label_num);
    result.confidence_ = detections[i * object_size_ + 2];
    if (result.confidence_ <= show_output_thresh_) {
      continue;
    }
    if (image_id < 0) {
      break;
    }
    found_result = true;
    results_.emplace_back(result);
  }
  if (!found_result) {
    results_.clear();
  }
  return true;
}

const int dynamic_vino_lib::ObjectDetection::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::ObjectDetection::Result *
dynamic_vino_lib::ObjectDetection::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::ObjectDetection::getName() const
{
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::ObjectDetection::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> dynamic_vino_lib::ObjectDetection::getFilteredROIs(
  const std::string filter_conditions) const
{
  result_filter_->acceptResults(results_);
  result_filter_->acceptFilterConditions(filter_conditions);
  return result_filter_->getFilteredLocations();
}


// ObjectDetectionResultFilter
dynamic_vino_lib::ObjectDetectionResultFilter::ObjectDetectionResultFilter() {}

void dynamic_vino_lib::ObjectDetectionResultFilter::init()
{
  key_to_function_.insert(std::make_pair("label", isValidLabel));
  key_to_function_.insert(std::make_pair("confidence", isValidConfidence));
}

void dynamic_vino_lib::ObjectDetectionResultFilter::acceptResults(
  const std::vector<Result> & results)
{
  results_ = results;
}

std::vector<cv::Rect>
dynamic_vino_lib::ObjectDetectionResultFilter::getFilteredLocations()
{
  std::vector<cv::Rect> locations;
  for (auto result : results_) {
    if (isValidResult(result)) {
      locations.push_back(result.getLocation());
    }
  }
  return locations;
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidLabel(
  const Result & result, const std::string & op, const std::string & target)
{
  if (!op.compare("==")) {
    return !target.compare(result.getLabel());
  } else if (!op.compare("!=")) {
    return target.compare(result.getLabel());
  } else {
    slog::err << "Invalid operator " << op << " for label comparsion" << slog::endl;
    return false;
  }
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidConfidence(
  const Result & result, const std::string & op, const std::string & target)
{
  float confidence = 0;
  try {
    confidence = std::stof(target);
  } catch (...) {
    slog::err << "Failed when converting string " << target << " to float" << slog::endl;
  }
  if (!op.compare("<=")) {
    return result.getConfidence() <= confidence;
  } else if (!op.compare(">=")) {
    return result.getConfidence() >= confidence;
  } else if (!op.compare("<")) {
    return result.getConfidence() < confidence;
  } else if (!op.compare(">")) {
    return result.getConfidence() > confidence;
  } else {
    slog::err << "Invalid operator " << op << " for confidence comparsion" << slog::endl;
    return false;
  }
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidResult(
  const Result & result)
{
  ISVALIDRESULT(key_to_function_, result);
}
