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
 * @brief A header file with declaration for BaseReidentification Class
 * @file base_reidentification.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__BASE_REIDENTIFICATION_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__BASE_REIDENTIFICATION_HPP_
#include <vector>
#include <cmath>
#include <string>
#include <thread>
#include <mutex>
#include <unordered_map>

// namespace
namespace openvino_wrapper_lib
{
/**
 * @class Tracker
 * @brief Class for storing and tracking the detected objects.
 */
class Tracker
{
public:
  explicit Tracker(int, double, double);
  /**
   * @brief Process the new detected track.
   * @param[in] feature The new detected track feature.
   * @return The detected track ID.
   */
  int processNewTrack(const std::vector<float> & feature);

private:
  /**
   * @brief Find the most similar track from the recorded tracks.
   * @param[in] feature The input track's feature.
   * @param[in] most similar track's ID to be recorded.
   * @return similarity with the most similar track.
   */
  double findMostSimilarTrack(const std::vector<float> & feature, int & most_similar_id);
  /**
   * @brief Update the matched track's feature by the new track.
   * @param[in] track_id The matched track ID.
   * @param[in] feature The matched track's feature
   */
  void updateMatchTrack(int track_id, const std::vector<float> & feature);
  /**
   * @brief Remove the earlest track from the recorded tracks.
   */
  void removeEarlestTrack();
  /**
   * @brief Add a new track to the recorded tracks, remove oldest track if needed.
   * @param[in] feature A track's feature.
   * @return new added track's ID.
   */
  int addNewTrack(const std::vector<float> & feature);
  /**
   * @brief Calculate the cosine similarity between two features.
   * @return The simlarity result.
   */
  double calcSimilarity(
    const std::vector<float> & feature_a, const std::vector<float> & feature_b);
  /**
   * @brief get the current millisecond count since epoch.
   * @return millisecond count since epoch.
   */
  int64_t getCurrentTime();

  bool saveTracksToFile(std::string filepath);
  bool loadTracksFromFile(std::string filepath);

  struct Track
  {
    int64_t lastest_update_time;
    std::vector<float> feature;
  };

  int max_record_size_ = 1000;
  int max_track_id_ = -1;
  double same_track_thresh_ = 0.9;
  double new_track_thresh_ = 0.3;
  std::mutex tracks_mtx_;
  std::unordered_map<int, Track> recorded_tracks_;
};

}  // namespace openvino_wrapper_lib
#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__BASE_REIDENTIFICATION_HPP_
