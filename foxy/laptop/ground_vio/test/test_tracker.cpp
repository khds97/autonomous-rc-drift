// test_tracker.cpp
// Tracks a synthetic textured image that moves 2 px per frame and checks ids and motion.
#include <cstdio>
#include <map>

#include <opencv2/imgproc.hpp>

#include "ground_vio/feature_tracker.hpp"
#include "test_common.hpp"

using namespace ground_vio;

int main()
{
  cv::RNG rng(3);
  cv::Mat noise(480, 900, CV_8UC1);
  rng.fill(noise, cv::RNG::UNIFORM, 0, 255);
  cv::Mat texture;
  cv::GaussianBlur(noise, texture, cv::Size(0, 0), 2.0);
  cv::normalize(texture, texture, 0, 255, cv::NORM_MINMAX);

  cv::Mat K = (cv::Mat_<double>(3, 3) << 400, 0, 320, 0, 400, 240, 0, 0, 1);
  cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
  FeatureTracker tracker(K, dist, TrackerParams());

  std::map<int64_t, Eigen::Vector2d> last;
  std::vector<double> steps;
  int max_age_frames = 0;
  std::map<int64_t, int> age;
  int last_count = 0;
  for (int f = 0; f < 25; ++f) {
    cv::Mat img = texture(cv::Rect(2 * f, 0, 640, 480)).clone();   // content moves left by 2 px
    const auto obs = tracker.track(img);
    std::map<int64_t, Eigen::Vector2d> now;
    for (const auto & o : obs) {
      now[o.id] = o.uv;
      age[o.id]++;
      max_age_frames = std::max(max_age_frames, age[o.id]);
      if (last.count(o.id)) {
        steps.push_back((o.uv - last[o.id]).x() * 400.0);
      }
    }
    last = now;
    last_count = static_cast<int>(obs.size());
  }
  double mean = 0.0;
  for (double s : steps) {
    mean += s;
  }
  mean /= std::max<size_t>(1, steps.size());
  std::printf(
    "tracked now %d, longest track %d frames, mean motion %.2f px per frame (expected -2)\n",
    last_count, max_age_frames, mean);
  CHECK(last_count > 60, "only %d features tracked", last_count);
  CHECK(max_age_frames >= 20, "longest track %d", max_age_frames);
  CHECK(std::abs(mean + 2.0) < 0.2, "mean motion %.2f", mean);
  return TEST_RESULT();
}
