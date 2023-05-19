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

/**
 * @brief a realization file with declaration of HumanPoseEstimation class and
 * HumanPoseEstimationResult class
 * @file human_pose_estimation.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "openvino_wrapper_lib/inferences/human_pose_estimation.hpp"
#include "openvino_wrapper_lib/outputs/base_output.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// HumanPoseEstimationResult
openvino_wrapper_lib::HumanPoseEstimationResult::HumanPoseEstimationResult(
  const cv::Rect & location)
: Result(location) {}

// HumanPoseEstimation
openvino_wrapper_lib::HumanPoseEstimation::HumanPoseEstimation()
: openvino_wrapper_lib::BaseInference() {}

openvino_wrapper_lib::HumanPoseEstimation::~HumanPoseEstimation() = default;
void openvino_wrapper_lib::HumanPoseEstimation::loadNetwork(
  const std::shared_ptr<Models::HumanPoseEstimationModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool openvino_wrapper_lib::HumanPoseEstimation::enqueue(
  const cv::Mat & frame, const cv::Rect & input_frame_loc)
{
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  auto model = valid_model_->getModel();
  
  ov::Shape inputShape = model->input().get_shape();
  const ov::Layout& layout = ov::layout::get_layout(model->inputs().front());
  const ov::Layout outputLayout("NCHW");
  const auto batchId = ov::layout::batch_idx(outputLayout);
  const auto heightId = ov::layout::height_idx(outputLayout);
  const auto widthId = ov::layout::width_idx(outputLayout);

  //if (!targetSize) {
  auto targetSize = inputShape[heightId];
  //}
  int stride = 8;
  //double aspectRatio = 1.77778;
  frame_size.width = frame.cols;
  frame_size.height = frame.rows;
  double aspectRatio = frame.cols / static_cast<double>(frame.rows);
  int height = static_cast<int>((targetSize + stride - 1) / stride) * stride;
  int inputWidth = static_cast<int>(std::round(targetSize * aspectRatio));
  int width = static_cast<int>((inputWidth + stride - 1) / stride) * stride;
  inputShape[batchId] = 1;
  inputShape[heightId] = height;
  inputShape[widthId] = width;
  auto inputLayerSize = cv::Size(width, height);
  model->reshape(inputShape);
  ///////////////
  if (!openvino_wrapper_lib::BaseInference::enqueue<u_int8_t>(
      frame, input_frame_loc, 1, 0, valid_model_->getInputName()))
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool openvino_wrapper_lib::HumanPoseEstimation::submitRequest()
{
  return openvino_wrapper_lib::BaseInference::submitRequest();
}

bool openvino_wrapper_lib::HumanPoseEstimation::fetchResults()
{
  bool can_fetch = openvino_wrapper_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;
  ov::InferRequest infer_request = getEngine()->getRequest();
  std::string heatmaps_name = valid_model_->getOutputName("heatmaps");
  std::string pafsOutput_name = valid_model_->getOutputName("pafsOutput");
  const auto& heatMapsMapped = infer_request.get_tensor(heatmaps_name);
  const auto& outputMapped = infer_request.get_tensor(pafsOutput_name);

  const ov::Shape& outputShape = outputMapped.get_shape();
  const ov::Shape& heatMapShape = heatMapsMapped.get_shape();
  float* const predictions = outputMapped.data<float>();
  float* const heats = heatMapsMapped.data<float>();
  //cv::Rect roi = cv::Rect(0, 0, frame_size.width, frame_size.height);
  cv::Rect roi = cv::Rect(0, 0, 0, 0);

  static const size_t keypointsNumber = 18;
  std::vector<cv::Mat> heatMaps(keypointsNumber);
  for (size_t i = 0; i < heatMaps.size(); i++) {
      heatMaps[i] = cv::Mat(heatMapShape[2], heatMapShape[3], CV_32FC1,
                            heats + i * heatMapShape[2] * heatMapShape[3]);
  }
  resizeFeatureMaps(heatMaps);

  std::vector<cv::Mat> pafs(outputShape[1]);
  for (size_t i = 0; i < pafs.size(); i++) {
      pafs[i] = cv::Mat(heatMapShape[2], heatMapShape[3], CV_32FC1,
                        predictions + i * heatMapShape[2] * heatMapShape[3]);
  }
  resizeFeatureMaps(pafs);

  std::vector<HumanPose> poses = extractPoses(heatMaps, pafs);

    float scaleX = stride / upsampleRatio * 1.68791; //res: 3.37582;
    float scaleY = stride / upsampleRatio * 1.6758; //res: 3.37582
    for (auto& pose : poses) {
        for (auto& keypoint : pose.keypoints) {
            if (keypoint != cv::Point2f(-1, -1)) {
                keypoint.x *= scaleX;
                keypoint.y *= scaleY;
            }
          //slog::info << "keypoint.x: " << keypoint.x<<", keypoint.y: "<< keypoint.y<<slog::endl;
        }
    }
    for (size_t i = 0; i < poses.size(); ++i) {
        results_[i].poses = poses[i];
        Result result(roi);
        result.poses = poses[i];
        results_.emplace_back(result);
        found_result = true;
    }

    if (!found_result) {
      results_.clear();
    }
    return true;
  
}

int openvino_wrapper_lib::HumanPoseEstimation::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const openvino_wrapper_lib::Result *
openvino_wrapper_lib::HumanPoseEstimation::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string openvino_wrapper_lib::HumanPoseEstimation::getName() const
{
  return valid_model_->getModelCategory();
}

void openvino_wrapper_lib::HumanPoseEstimation::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}

const std::vector<cv::Rect> openvino_wrapper_lib::HumanPoseEstimation::getFilteredROIs(
  const std::string filter_conditions) const
{
  if (!filter_conditions.empty()) {
    slog::err << "Human pose estimation does not support filtering now! " <<
      "Filter conditions: " << filter_conditions << slog::endl;
  }
  std::vector<cv::Rect> filtered_rois;
  for (auto res : results_) {
    filtered_rois.push_back(res.getLocation());
  }
  return filtered_rois;
}

void openvino_wrapper_lib::HumanPoseEstimation::resizeFeatureMaps(
  std::vector<cv::Mat>& featureMaps) 
{
    for (auto& featureMap : featureMaps) {
        cv::resize(
          featureMap, featureMap, cv::Size(), upsampleRatio, upsampleRatio, cv::INTER_CUBIC
        );
    }
}


std::vector<HumanPose> openvino_wrapper_lib::HumanPoseEstimation::extractPoses(
  const std::vector<cv::Mat>& heatMaps,
  const std::vector<cv::Mat>& pafs) 
{
    std::vector<std::vector<Peak>> peaksFromHeatMap(heatMaps.size());
    FindPeaksBody findPeaksBody(heatMaps, minPeaksDistance, peaksFromHeatMap, confidenceThreshold);
    cv::parallel_for_(cv::Range(0, static_cast<int>(heatMaps.size())), findPeaksBody);
    int peaksBefore = 0;
    for (size_t heatmapId = 1; heatmapId < heatMaps.size(); heatmapId++) {
        peaksBefore += static_cast<int>(peaksFromHeatMap[heatmapId - 1].size());
        for (auto& peak : peaksFromHeatMap[heatmapId]) {
            peak.id += peaksBefore;
        }
    }
    std::vector<HumanPose> poses = groupPeaksToPoses(peaksFromHeatMap,
                                                     pafs,
                                                     keypointsNumber,
                                                     midPointsScoreThreshold,
                                                     foundMidPointsRatioThreshold,
                                                     minJointsNumber,
                                                     minSubsetScore);                                             
    return poses;
}
