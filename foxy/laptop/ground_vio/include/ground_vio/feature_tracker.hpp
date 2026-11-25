// feature_tracker.hpp
// KLT feature tracker for the fixed camera. It returns the tracked features of each image in
// normalized (undistorted) coordinates, with an id that stays the same along a track.
#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "ground_vio/vio_filter.hpp"

namespace ground_vio
{

struct TrackerParams
{
  int max_features = 150;
  double min_distance = 25.0;       // pixels between features
  double quality_level = 0.01;
  int window_size = 21;
  int pyramid_levels = 3;
  double backward_error_px = 1.0;   // forward-backward flow check
  double ransac_threshold_px = 1.0; // epipolar outlier rejection
  int border_px = 10;
};

class FeatureTracker
{
public:
  // camera_matrix is 3x3 (CV_64F), distortion has 4 or 5 OpenCV coefficients (can be empty).
  FeatureTracker(
    const cv::Mat & camera_matrix, const cv::Mat & distortion, const TrackerParams & params);

  // gray must be CV_8UC1.
  std::vector<FeatureObservation> track(const cv::Mat & gray);

  void reset();
  int trackedCount() const {return static_cast<int>(points_.size());}

private:
  std::vector<cv::Point2f> undistortedPixels(const std::vector<cv::Point2f> & pts) const;
  std::vector<cv::Point2f> normalized(const std::vector<cv::Point2f> & pts) const;

  cv::Mat K_;
  cv::Mat dist_;
  TrackerParams params_;
  cv::Mat prev_;
  std::vector<cv::Point2f> points_;
  std::vector<int64_t> ids_;
  int64_t next_id_ = 0;
};

}  // namespace ground_vio
