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
 * @brief a header file with declaration of BaseReidentification class and
 * BaseReidentificationResult class
 * @file base_reidentification.cpp
 */
#include "dynamic_vino_lib/inferences/base_reidentification.hpp"

// Tracker
dynamic_vino_lib::Tracker::Tracker(
  int lost_track_thresh, double same_track_thresh, double new_track_thresh)
: lost_track_thresh_(lost_track_thresh),
  same_track_thresh_(same_track_thresh),
  new_track_thresh_(new_track_thresh) {}

int dynamic_vino_lib::Tracker::processNewTrack(const std::vector<float>& feature)
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

int dynamic_vino_lib::Tracker::findMatchTrack(const std::vector<float>& feature)
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

double dynamic_vino_lib::Tracker::calcSimilarity(
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

void dynamic_vino_lib::Tracker::updateMatchTrack(
  int track_id, const std::vector<float>& feature) {
  if (recorded_tracks_.find(track_id) != recorded_tracks_.end()) {
    recorded_tracks_[track_id].feature.assign(feature.begin(), feature.end());
    recorded_tracks_[track_id].lost = 0;
  }
  else {
    throw std::logic_error("updating a non-existing track.");
  }
}

void dynamic_vino_lib::Tracker::updateAllTracksLost() {
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    iter->second.lost += 1;
  }
}

void dynamic_vino_lib::Tracker::removeOldTracks() {
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    if (iter->second.lost > lost_track_thresh_) {
      recorded_tracks_.erase(iter);
    }
  }
}

void dynamic_vino_lib::Tracker::addNewTrack(const std::vector<float>& feature) {
  Track track;
  track.lost = 0;
  track.feature.assign(feature.begin(), feature.end());
  max_track_id_ += 1;
  recorded_tracks_.insert(std::pair<int, Track>(max_track_id_, track));
}
