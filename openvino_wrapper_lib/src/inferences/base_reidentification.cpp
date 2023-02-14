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
 * @brief a header file with declaration of BaseReidentification class
 * @file base_reidentification.cpp
 */
#include <vector>
#include <utility>
#include <climits>
#include <chrono>
#include <fstream>
#include <string>
#include <functional>
#include "openvino_wrapper_lib/inferences/base_reidentification.hpp"
#include "openvino_wrapper_lib/slog.hpp"

// Tracker
openvino_wrapper_lib::Tracker::Tracker(
  int max_record_size, double same_track_thresh, double new_track_thresh)
: max_record_size_(max_record_size),
  same_track_thresh_(same_track_thresh),
  new_track_thresh_(new_track_thresh) {}

int openvino_wrapper_lib::Tracker::processNewTrack(const std::vector<float> & feature)
{
  int most_similar_id;
  double similarity = findMostSimilarTrack(feature, most_similar_id);
  if (similarity > same_track_thresh_) {
    updateMatchTrack(most_similar_id, feature);
  } else if (similarity < new_track_thresh_) {
    most_similar_id = addNewTrack(feature);
  }
  return most_similar_id;
}

double openvino_wrapper_lib::Tracker::findMostSimilarTrack(
  const std::vector<float> & feature, int & most_similar_id)
{
  double max_similarity = 0;
  most_similar_id = -1;
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    double sim = calcSimilarity(feature, iter->second.feature);
    if (sim > max_similarity) {
      max_similarity = sim;
      most_similar_id = iter->first;
    }
  }
  return max_similarity;
}

double openvino_wrapper_lib::Tracker::calcSimilarity(
  const std::vector<float> & feature_a, const std::vector<float> & feature_b)
{
  if (feature_a.size() != feature_b.size()) {
    slog::err << "cosine similarity can't be called for vectors of different lengths: " <<
      "feature_a size = " << std::to_string(feature_a.size()) <<
      "feature_b size = " << std::to_string(feature_b.size()) << slog::endl;
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
    slog::err << "cosine similarity is not defined whenever one or both "
      "input vectors are zero-vectors." << slog::endl;
  }
  return mul_sum / (sqrt(denom_a) * sqrt(denom_b));
}

void openvino_wrapper_lib::Tracker::updateMatchTrack(
  int track_id, const std::vector<float> & feature)
{
  if (recorded_tracks_.find(track_id) != recorded_tracks_.end()) {
    recorded_tracks_[track_id].feature.assign(feature.begin(), feature.end());
    recorded_tracks_[track_id].lastest_update_time = getCurrentTime();
  } else {
    slog::err << "updating a non-existing track." << slog::endl;
  }
}

void openvino_wrapper_lib::Tracker::removeEarlestTrack()
{
  std::lock_guard<std::mutex> lk(tracks_mtx_);
  int64_t earlest_time = LONG_MAX;
  auto remove_iter = recorded_tracks_.begin();
  for (auto iter = recorded_tracks_.begin(); iter != recorded_tracks_.end(); iter++) {
    if (iter->second.lastest_update_time < earlest_time) {
      earlest_time = iter->second.lastest_update_time;
      remove_iter = iter;
    }
  }
  recorded_tracks_.erase(remove_iter);
}


int openvino_wrapper_lib::Tracker::addNewTrack(const std::vector<float> & feature)
{
  if (recorded_tracks_.size() >= max_record_size_) {
    std::thread remove_thread(std::bind(&Tracker::removeEarlestTrack, this));
    remove_thread.detach();
  }
  std::lock_guard<std::mutex> lk(tracks_mtx_);
  Track track;
  track.lastest_update_time = getCurrentTime();
  track.feature.assign(feature.begin(), feature.end());
  max_track_id_ += 1;
  recorded_tracks_.insert(std::pair<int, Track>(max_track_id_, track));
  return max_track_id_;
}

int64_t openvino_wrapper_lib::Tracker::getCurrentTime()
{
  auto tp = std::chrono::time_point_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now());
  return static_cast<int64_t>(tp.time_since_epoch().count());
}

bool openvino_wrapper_lib::Tracker::saveTracksToFile(std::string filepath)
{
  std::ofstream outfile(filepath);
  if (!outfile.is_open()) {
    slog::err << "file not exists in file path: " << filepath << slog::endl;
    return false;
  }
  for (auto record : recorded_tracks_) {
    outfile << record.first << " " <<
      record.second.lastest_update_time << " ";
    for (auto elem : record.second.feature) {
      outfile << elem << " ";
    }
    outfile << "\n";
  }
  outfile.close();
  slog::info << "sucessfully save tracks into file: " << filepath << slog::endl;
  return true;
}

bool openvino_wrapper_lib::Tracker::loadTracksFromFile(std::string filepath)
{
  std::ifstream infile(filepath);
  if (!infile.is_open()) {
    slog::err << "file not exists in file path: " << filepath << slog::endl;
    return false;
  }
  recorded_tracks_.clear();
  while (!infile.eof()) {
    int track_id;
    int64_t lastest_update_time;
    std::vector<float> feature;
    infile >> track_id >> lastest_update_time;
    for (int num = 0; num < 256; num++) {
      float elem;
      infile >> elem;
      feature.push_back(elem);
    }
    Track track;
    track.lastest_update_time = lastest_update_time;
    track.feature = feature;
    recorded_tracks_[track_id] = track;
  }
  infile.close();
  slog::info << "sucessfully load tracks from file: " << filepath << slog::endl;
  return true;
}
