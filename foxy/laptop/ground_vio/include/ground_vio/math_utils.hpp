// math_utils.hpp
// Small rotation helpers shared by the filter and the tests.
#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ground_vio
{

inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

// Quaternion of the rotation vector w (axis * angle).
inline Eigen::Quaterniond expQuat(const Eigen::Vector3d & w)
{
  const double angle = w.norm();
  if (angle < 1e-9) {
    return Eigen::Quaterniond(1.0, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, w / angle));
}

inline Eigen::Matrix3d expMat(const Eigen::Vector3d & w)
{
  return expQuat(w).toRotationMatrix();
}

// Rotation vector of a quaternion (inverse of expQuat).
inline Eigen::Vector3d logQuat(const Eigen::Quaterniond & q)
{
  Eigen::Quaterniond n = q.normalized();
  if (n.w() < 0.0) {
    n.coeffs() = -n.coeffs();
  }
  const Eigen::Vector3d v = n.vec();
  const double s = v.norm();
  if (s < 1e-12) {
    return 2.0 * v;
  }
  return 2.0 * std::atan2(s, n.w()) * v / s;
}

}  // namespace ground_vio
