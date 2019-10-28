#ifndef OPENVINO__TRACKER_HPP_
#define OPENVINO__TRACKER_HPP_

#include <vector>
#include <cmath>
#include <string>
#include <thread>
#include <mutex>
#include <unordered_map>

namespace openvino
{
class Tracker
{
public:
  explicit Tracker(int, double, double);
  int processNewTracker(const std::vector<float> & feature);

private:
  double findMostSimilarTrack(const std::vector<float> & feature, int & most_similar_id);
  void updateMatchTrack(int track_id, const std::vector<float> & feature);
  void removeEarlestTrack();
  int addNewTrack(const std::vector<float> & feature);
  double calcSimilarity(
    const std::vector<float> & feature_a, const std::vector<float> & feature_b);

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
}  // namespace openvino

#endif  // OPENVINO__TRACKER_HPP_
