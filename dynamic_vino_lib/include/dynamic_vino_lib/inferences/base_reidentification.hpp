

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
 * @brief A header file with declaration for BaseReidentification Class
 * @file base_reidentification.hpp
 */
#ifndef DYNAMIC_VINO_LIB__INFERENCES__BASE_REIDENTIFICATION_HPP_
#define DYNAMIC_VINO_LIB__INFERENCES__BASE_REIDENTIFICATION_HPP_
#include <vector>
#include <cmath>
#include <unordered_map>

// namespace
namespace dynamic_vino_lib
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
  int processNewTrack(const std::vector<float>& feature);

private:
  /**
   * @brief Find the matched track from the recorded tracks.
   * @param[in] feature The input track's feature.
   * @return The matched track ID, -1 if it's not matched.
   */
  int findMatchTrack(const std::vector<float>& feature);
  /**
   * @brief Update the matched track's feature by the new track.
   * @param[in] track_id The matched track ID.
   * @param[in] feature The matched track's feature
   */
  void updateMatchTrack(int track_id, const std::vector<float>& feature);
  /**
   * @brief Update the recorded tracks' lost.
   */
  void updateAllTracksLost();
  /**
   * @brief Remove the old tracks from the recorded tracks.
   */
  void removeOldTracks();
  /**
   * @brief Add a new track to the recorded tracks.
   * @param[in] feature A track's feature.
   */
  void addNewTrack(const std::vector<float>& feature);
  /**
   * @brief Calculate the cosine similarity between two features.
   * @return The simlarity result.
   */
  double calcSimilarity(
    const std::vector<float> & feature_a, const std::vector<float> & feature_b);

  struct Track
  {
    int lost;
    std::vector<float> feature;
  };
  
  int lost_track_thresh_ = 10000;
  int max_track_id_ = -1;
  double max_match_similarity_ = 0;
  double same_track_thresh_ = 0.9;
  double new_track_thresh_ = 0.3;
  std::unordered_map<int, Track> recorded_tracks_;
};

}  // namespace dynamic_vino_lib
#endif  // DYNAMIC_VINO_LIB__INFERENCES__BASE_REIDENTIFICATION_HPP_
