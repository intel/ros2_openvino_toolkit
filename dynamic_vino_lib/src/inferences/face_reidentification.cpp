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
 * @brief a header file with declaration of FaceReidentification class and
 * FaceReidentificationResult class
 * @file face_reidentification.cpp
 */
#include <memory>
#include <string>
#include <vector>
#include "dynamic_vino_lib/inferences/face_reidentification.hpp"
#include "dynamic_vino_lib/outputs/base_output.hpp"
#include "dynamic_vino_lib/slog.hpp"

// FaceReidentificationResult
dynamic_vino_lib::FaceReidentificationResult::FaceReidentificationResult(
  const cv::Rect & location)
: Result(location) {}

// FaceTracker
dynamic_vino_lib::FaceTracker::FaceTracker(
  int drop_lost_thresh, double same_track_thresh, double new_track_thresh)
: drop_lost_thresh_(drop_lost_thresh),
  same_track_thresh_(same_track_thresh),
  new_track_thresh_(new_track_thresh) {}


int dynamic_vino_lib::FaceTracker::processNewTrack(const std::vector<float>& feature)
{
  int track_id = findMatchTrack(feature);
  updateAllTracksLost();
  if (track_id >= 0 and max_match_similarity_ > same_track_thresh_) {
    updateMatchTrack(track_id, feature);
  }
  else if (max_match_similarity_ < new_track_thresh_) {
    addNewTrack(feature);
    track_id = max_track_id_;
  }
  removeOldTracks();
  return track_id;
}

int dynamic_vino_lib::FaceTracker::findMatchTrack(const std::vector<float>& feature)
{
  double max_sim = 0;
  int match_track_id = -1;
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    double sim = calcSimilarity(feature, iter->second.feature);
    if (sim > max_sim) {
      max_sim = sim;
      match_track_id = iter->first;
    }
  }
  max_match_similarity_ = max_sim;
  return match_track_id;
}

double dynamic_vino_lib::FaceTracker::calcSimilarity(
  const std::vector<float> & feature_a, const std::vector<float> & feature_b) {
  if (feature_a.size() != feature_b.size()) {
    throw std::logic_error("cosine similarity can't be called for vectors of different lengths: "
            "feature_a size = " + std::to_string(feature_a.size()) +
            "feature_b size = " + std::to_string(feature_b.size()));
  }
  float mul_sum, denom_a, denom_b, value_a, value_b;
  mul_sum = denom_a = denom_b = value_a = value_b = 0;
  for (auto i = 0; i < feature_a.size(); i++) {
    value_a = feature_a[i];
    value_b = feature_b[i];
    mul_sum += value_a * value_b;
    denom_a += value_a * value_a;
    denom_b += value_b * value_b;
  }
  if (denom_a == 0 || denom_b == 0) {
    throw std::logic_error("cosine similarity is not defined whenever one or both "
            "input vectors are zero-vectors.");
  }
  return mul_sum / (sqrt(denom_a) * sqrt(denom_b));
}

void dynamic_vino_lib::FaceTracker::updateMatchTrack(
  int track_id, const std::vector<float>& feature) {
  if (recorded_tracks_.find(track_id) != recorded_tracks_.end()) {
    recorded_tracks_[track_id].feature.assign(feature.begin(), feature.end());
    recorded_tracks_[track_id].lost = 0;
  }
  else {
    throw std::logic_error("updating a non-existing track.");
  }
}

void dynamic_vino_lib::FaceTracker::updateAllTracksLost() {
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    iter->second.lost += 1;
  }
}

void dynamic_vino_lib::FaceTracker::removeOldTracks() {
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    if (iter->second.lost > drop_lost_thresh_) {
      recorded_tracks_.erase(iter);
    }
  }
}

void dynamic_vino_lib::FaceTracker::addNewTrack(const std::vector<float>& feature) {
  Track track;
  track.lost = 0;
  track.feature.assign(feature.begin(), feature.end());
  max_track_id_ += 1;
  recorded_tracks_.insert(std::pair<int, Track>(max_track_id_, track));
}

// FaceReidentification
dynamic_vino_lib::FaceReidentification::FaceReidentification(double match_thresh)
: match_thresh_(match_thresh), dynamic_vino_lib::BaseInference() {
  face_tracker_ = std::make_shared<dynamic_vino_lib::FaceTracker>(1000000, 0.9, 0.3);
}

dynamic_vino_lib::FaceReidentification::~FaceReidentification() = default;
void dynamic_vino_lib::FaceReidentification::loadNetwork(
  const std::shared_ptr<Models::FaceReidentificationModel> network)
{
  valid_model_ = network;
  setMaxBatchSize(network->getMaxBatchSize());
}

bool dynamic_vino_lib::FaceReidentification::enqueue(
  const cv::Mat & frame, const cv::Rect & input_frame_loc)
{
  if (getEnqueuedNum() == 0) {
    results_.clear();
  }
  if (!dynamic_vino_lib::BaseInference::enqueue<u_int8_t>(
      frame, input_frame_loc, 1, 0, valid_model_->getInputName()))
  {
    return false;
  }
  Result r(input_frame_loc);
  results_.emplace_back(r);
  return true;
}

bool dynamic_vino_lib::FaceReidentification::submitRequest()
{
  return dynamic_vino_lib::BaseInference::submitRequest();
}

bool dynamic_vino_lib::FaceReidentification::fetchResults()
{
  bool can_fetch = dynamic_vino_lib::BaseInference::fetchResults();
  if (!can_fetch) {return false;}
  bool found_result = false;
  InferenceEngine::InferRequest::Ptr request = getEngine()->getRequest();
  std::string output = valid_model_->getOutputName();
  const float * output_values = request->GetBlob(output)->buffer().as<float *>();
  int result_length = request->GetBlob(output)->getTensorDesc().getDims()[1];
  for (int i = 0; i < getResultsLength(); i++) {
    std::vector<float> new_face = std::vector<float>(
      output_values + result_length * i, output_values + result_length * (i + 1));
    std::string face_id = "No." + std::to_string(face_tracker_->processNewTrack(new_face));
    // std::string face_id = findMatchFace(new_face);
    results_[i].face_id_ = face_id;
    found_result = true;
  }
  if (!found_result) {results_.clear();}
  return true;
}


float dynamic_vino_lib::FaceReidentification::calcSimilarity(
  const std::vector<float> & face_a, const std::vector<float> & face_b)
{
  if (face_a.size() != face_b.size()) {
    throw std::logic_error("cosine similarity can't be called for vectors of different lengths: "
            "face_a size = " + std::to_string(face_a.size()) +
            "face_b size = " + std::to_string(face_b.size()));
  }
  float mul_sum, denom_a, denom_b, value_a, value_b;
  mul_sum = denom_a = denom_b = value_a = value_b = 0;
  for (auto i = 0; i < face_a.size(); i++) {
    value_a = face_a[i];
    value_b = face_b[i];
    mul_sum += value_a * value_b;
    denom_a += value_a * value_a;
    denom_b += value_b * value_b;
  }
  if (denom_a == 0 || denom_b == 0) {
    throw std::logic_error("cosine similarity is not defined whenever one or both "
            "input vectors are zero-vectors.");
  }
  return mul_sum / (sqrt(denom_a) * sqrt(denom_b));
}

std::string dynamic_vino_lib::FaceReidentification::findMatchFace(
  const std::vector<float> & new_face)
{
  auto size = recorded_faces_.size();
  std::string id = "No.";
  float best_match_sim = 0;
  int best_match_ind = -1;
  for (auto i = 0; i < size; ++i) {
    float cos_sim = calcSimilarity(new_face, recorded_faces_[i]);
    if (cos_sim > best_match_sim) {
      best_match_sim = cos_sim;
      best_match_ind = i;
    }
  }
  std::cout<<best_match_sim<<std::endl;
  if (best_match_sim > match_thresh_) {
    recorded_faces_[best_match_ind] = new_face;
    return id + std::to_string(best_match_ind);
  } else {
    std::cout<<"New Face Detected, probability is:"<<best_match_sim<<std::endl;
    recorded_faces_.push_back(new_face);
    return id + std::to_string(size);
  }
}

const int dynamic_vino_lib::FaceReidentification::getResultsLength() const
{
  return static_cast<int>(results_.size());
}

const dynamic_vino_lib::Result *
dynamic_vino_lib::FaceReidentification::getLocationResult(int idx) const
{
  return &(results_[idx]);
}

const std::string dynamic_vino_lib::FaceReidentification::getName() const
{
  return valid_model_->getModelName();
}

const void dynamic_vino_lib::FaceReidentification::observeOutput(
  const std::shared_ptr<Outputs::BaseOutput> & output)
{
  if (output != nullptr) {
    output->accept(results_);
  }
}
