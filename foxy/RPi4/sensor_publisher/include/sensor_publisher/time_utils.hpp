// time_utils.hpp
// Converts kernel event times (GPIO edges, V4L2 frames) to the ROS clock.
#pragma once

#include <cstdint>
#include <time.h>

#include "rclcpp/rclcpp.hpp"

namespace sensor_publisher
{

inline int64_t clockNs(clockid_t id)
{
  timespec t;
  clock_gettime(id, &t);
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

// Age of a kernel event time in nanoseconds (now minus the event time).
// Depending on the driver and kernel, event times are on CLOCK_REALTIME or
// CLOCK_MONOTONIC. Both are compared with the current time, and the one that gives
// an age between -5 ms and max_age_ns is used.
// Returns false if neither clock gives a plausible age.
inline bool kernelEventAgeNs(int64_t kernel_ns, int64_t max_age_ns, int64_t * age_ns)
{
  constexpr int64_t kMinAgeNs = -5000000;

  const int64_t real_age = clockNs(CLOCK_REALTIME) - kernel_ns;
  if (real_age >= kMinAgeNs && real_age <= max_age_ns) {
    *age_ns = real_age;
    return true;
  }

  const int64_t mono_age = clockNs(CLOCK_MONOTONIC) - kernel_ns;
  if (mono_age >= kMinAgeNs && mono_age <= max_age_ns) {
    *age_ns = mono_age;
    return true;
  }

  return false;
}

// Kernel event time as a time on the given ROS clock. If the event time is not
// usable, the current time is returned and *used_kernel_time is set to false.
inline rclcpp::Time kernelEventToRosTime(
  rclcpp::Clock & clock, int64_t kernel_ns, int64_t max_age_ns,
  bool * used_kernel_time = nullptr)
{
  const rclcpp::Time now = clock.now();

  int64_t age_ns = 0;
  const bool ok = kernelEventAgeNs(kernel_ns, max_age_ns, &age_ns);
  if (used_kernel_time != nullptr) {
    *used_kernel_time = ok;
  }
  if (!ok) {
    return now;
  }
  return rclcpp::Time(now.nanoseconds() - age_ns, now.get_clock_type());
}

}  // namespace sensor_publisher
