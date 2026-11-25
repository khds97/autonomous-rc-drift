// feature_tracker.cpp
#include "ground_vio/feature_tracker.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace ground_vio
{

FeatureTracker::FeatureTracker(
  const cv::Mat & camera_matrix, const cv::Mat & distortion, const TrackerParams & params)
: K_(camera_matrix.clone()), dist_(distortion.clone()), params_(params)
{
}

void FeatureTracker::reset()
{
  prev_.release();
  points_.clear();
  ids_.clear();
}

std::vector<cv::Point2f> FeatureTracker::undistortedPixels(
  const std::vector<cv::Point2f> & pts) const
{
  std::vector<cv::Point2f> out;
  if (pts.empty()) {
    return out;
  }
  cv::undistortPoints(pts, out, K_, dist_, cv::noArray(), K_);
  return out;
}

std::vector<cv::Point2f> FeatureTracker::normalized(const std::vector<cv::Point2f> & pts) const
{
  std::vector<cv::Point2f> out;
  if (pts.empty()) {
    return out;
  }
  cv::undistortPoints(pts, out, K_, dist_);
  return out;
}

std::vector<FeatureObservation> FeatureTracker::track(const cv::Mat & gray)
{
  CV_Assert(gray.type() == CV_8UC1);
  const cv::Size win(params_.window_size, params_.window_size);
  const cv::TermCriteria term(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);

  // 1. Follow the existing features into the new image.
  if (!prev_.empty() && !points_.empty()) {
    std::vector<cv::Point2f> next, back;
    std::vector<uchar> st_fwd, st_bwd;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
      prev_, gray, points_, next, st_fwd, err, win, params_.pyramid_levels, term);
    back = points_;
    cv::calcOpticalFlowPyrLK(
      gray, prev_, next, back, st_bwd, err, win, params_.pyramid_levels, term,
      cv::OPTFLOW_USE_INITIAL_FLOW);

    std::vector<cv::Point2f> kept_prev, kept_next;
    std::vector<int64_t> kept_ids;
    for (size_t i = 0; i < points_.size(); ++i) {
      if (!st_fwd[i] || !st_bwd[i]) {
        continue;
      }
      const cv::Point2f d = back[i] - points_[i];
      if (std::hypot(d.x, d.y) > params_.backward_error_px) {
        continue;
      }
      const cv::Point2f & p = next[i];
      if (p.x < params_.border_px || p.y < params_.border_px ||
        p.x > gray.cols - params_.border_px || p.y > gray.rows - params_.border_px)
      {
        continue;
      }
      kept_prev.push_back(points_[i]);
      kept_next.push_back(next[i]);
      kept_ids.push_back(ids_[i]);
    }

    // 2. Drop tracks that disagree with the epipolar geometry of the camera motion.
    if (kept_next.size() >= 12) {
      std::vector<uchar> inlier;
      cv::findFundamentalMat(
        undistortedPixels(kept_prev), undistortedPixels(kept_next), cv::FM_RANSAC,
        params_.ransac_threshold_px, 0.99, inlier);
      std::vector<cv::Point2f> pts;
      std::vector<int64_t> ids;
      for (size_t i = 0; i < kept_next.size(); ++i) {
        if (inlier.empty() || inlier[i]) {
          pts.push_back(kept_next[i]);
          ids.push_back(kept_ids[i]);
        }
      }
      points_ = pts;
      ids_ = ids;
    } else {
      points_ = kept_next;
      ids_ = kept_ids;
    }
  } else {
    points_.clear();
    ids_.clear();
  }

  // 3. Add new features where there are none yet.
  const int missing = params_.max_features - static_cast<int>(points_.size());
  if (missing > 0) {
    cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar(255));
    for (const auto & p : points_) {
      cv::circle(mask, p, static_cast<int>(params_.min_distance), cv::Scalar(0), -1);
    }
    cv::rectangle(
      mask, cv::Rect(0, 0, gray.cols, gray.rows), cv::Scalar(0), params_.border_px);
    std::vector<cv::Point2f> fresh;
    cv::goodFeaturesToTrack(
      gray, fresh, missing, params_.quality_level, params_.min_distance, mask);
    for (const auto & p : fresh) {
      points_.push_back(p);
      ids_.push_back(next_id_++);
    }
  }

  gray.copyTo(prev_);

  std::vector<FeatureObservation> out;
  const std::vector<cv::Point2f> norm = normalized(points_);
  out.reserve(norm.size());
  for (size_t i = 0; i < norm.size(); ++i) {
    FeatureObservation o;
    o.id = ids_[i];
    o.uv = Eigen::Vector2d(norm[i].x, norm[i].y);
    out.push_back(o);
  }
  return out;
}

}  // namespace ground_vio
