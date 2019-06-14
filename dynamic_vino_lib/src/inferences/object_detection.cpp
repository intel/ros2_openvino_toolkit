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
  std::shared_ptr<Models::ObjectDetectionModel> network)
{
  valid_model_ = network;

  setMaxBatchSize(network->getMaxBatchSize());
}
bool dynamic_vino_lib::ObjectDetection::enqueue(
  const cv::Mat & frame,
  const cv::Rect & input_frame_loc)
{
    if(valid_model_ == nullptr || getEngine() == nullptr){
      return false;
    }

    if (enqueued_frames_ >= valid_model_->getMaxBatchSize())
  {
    slog::warn << "Number of " << getName() << "input more than maximum("
               << max_batch_size_ << ") processed by inference" << slog::endl;
    return false;
  }

    if(!valid_model_->enqueue(getEngine(), frame, input_frame_loc)){
      return false;
    }

    //nonsense!!
    //Result r(input_frame_loc);
    //results_.clear();
    //results_.emplace_back(r);
    enqueued_frames_ += 1;
    return true;
}

bool dynamic_vino_lib::ObjectDetection::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) {
    return false;
  }

  results_.clear();

  return (valid_model_ != nullptr) && valid_model_->fetchResults(getEngine(), results_, show_output_thresh_, enable_roi_constraint_);
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
  return stringCompare(result.getLabel(), op, target);
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidConfidence(
  const Result & result, const std::string & op, const std::string & target)
{
  return floatCompare(result.getConfidence(), op, stringToFloat(target));
}

bool dynamic_vino_lib::ObjectDetectionResultFilter::isValidResult(
  const Result & result)
{
  ISVALIDRESULT(key_to_function_, result);
}

double dynamic_vino_lib::ObjectDetection::IntersectionOverUnion(const cv::Rect &box_1, const cv::Rect &box_2)
{
  int xmax_1 = box_1.x + box_1.width;
  int xmin_1 = box_1.x;
  int xmax_2 = box_2.x + box_2.width;
  int xmin_2 = box_2.x;

  int ymax_1 = box_1.y + box_1.height;
  int ymin_1 = box_1.y;
  int ymax_2 = box_2.y + box_2.height;
  int ymin_2 = box_2.y;

  double width_of_overlap_area = fmin(xmax_1 , xmax_2) - fmax(xmin_1, xmin_2);
  double height_of_overlap_area = fmin(ymax_1, ymax_2) - fmax(ymin_1, ymin_2);
  double area_of_overlap;
  if (width_of_overlap_area < 0 || height_of_overlap_area < 0)
    area_of_overlap = 0;
  else
    area_of_overlap = width_of_overlap_area * height_of_overlap_area;

 double box_1_area = (ymax_1 - ymin_1)  * (xmax_1 - xmin_1);
 double box_2_area = (ymax_2 - ymin_2)  * (xmax_2 - xmin_2);
 double area_of_union = box_1_area + box_2_area - area_of_overlap;

 return area_of_overlap / area_of_union;
}
