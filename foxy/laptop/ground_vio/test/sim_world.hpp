// sim_world.hpp
// Synthetic trajectory, landmarks, IMU and camera used to test the filter without hardware.
#pragma once

#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ground_vio/math_utils.hpp"
#include "ground_vio/vio_filter.hpp"

namespace sim
{
using ground_vio::expQuat;

struct Trajectory
{
  bool planar = false;
  double still_s = 2.0;   // the vehicle stands still, then speeds up smoothly

  // Progress along the path; zero while standing still.
  double tau(double t) const
  {
    const double x = (t - still_s) / 2.0;
    if (x <= 0.0) {
      return 0.0;
    }
    if (x < 1.0) {
      return 2.0 * (std::pow(x, 6) - 3.0 * std::pow(x, 5) + 2.5 * std::pow(x, 4));
    }
    return 2.0 * (0.5 + (x - 1.0));
  }

  Eigen::Vector3d position(double t) const
  {
    const double s = tau(t);
    const double z = planar ? 0.0 : 0.06 * std::sin(0.9 * s);
    return Eigen::Vector3d(0.8 * s + 1.2 * std::sin(0.3 * s), 1.5 * std::sin(0.25 * s), z);
  }

  Eigen::Quaterniond orientation(double t) const
  {
    const double s = tau(t);
    const double yaw = 0.7 * std::sin(0.35 * s) + 0.2 * std::sin(0.9 * s);
    const double pitch = planar ? 0.0 : 0.04 * std::sin(0.7 * s);
    const double roll = planar ? 0.0 : 0.05 * std::sin(1.3 * s);
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) *
           Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) *
           Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  }

  // Body angular rate and world acceleration by numerical differentiation.
  Eigen::Vector3d bodyRate(double t) const
  {
    const double h = 1e-4;
    const Eigen::Quaterniond q0 = orientation(t - h);
    const Eigen::Quaterniond q1 = orientation(t + h);
    return ground_vio::logQuat(q0.conjugate() * q1) / (2.0 * h);
  }

  Eigen::Vector3d acceleration(double t) const
  {
    const double h = 1e-3;
    return (position(t + h) - 2.0 * position(t) + position(t - h)) / (h * h);
  }
};

struct Camera
{
  double f = 400.0;
  double cx = 320.0;
  double cy = 240.0;
  int width = 640;
  int height = 480;
};

struct World
{
  Trajectory traj;
  Camera cam;
  Eigen::Quaterniond q_IC;   // true camera-in-IMU pose
  Eigen::Vector3d p_IC;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  Eigen::Vector3d bg = Eigen::Vector3d(0.002, -0.001, 0.0015);
  Eigen::Vector3d ba = Eigen::Vector3d(0.05, -0.03, 0.04);
  double gyro_noise = 1e-4;    // continuous densities used to generate the data
  double accel_noise = 1e-3;
  double pixel_noise = 0.5;
  double outlier_rate = 0.015;
  std::vector<Eigen::Vector3d> landmarks;
  std::mt19937 rng{42};        // landmarks
  std::mt19937 rng_imu{7};
  std::mt19937 rng_cam{11};

  World()
  {
    // IMU x forward, y left, z up; camera z forward, x right, y down.
    Eigen::Matrix3d R;
    R << 0, 0, 1,
      -1, 0, 0,
      0, -1, 0;
    q_IC = Eigen::Quaterniond(R);
    p_IC = Eigen::Vector3d(0.10, 0.0, 0.08);
    std::uniform_real_distribution<double> ux(-6.0, 48.0), uy(-8.0, 10.0), uz(-0.15, 2.5);
    for (int i = 0; i < 4000; ++i) {
      landmarks.emplace_back(ux(rng), uy(rng), -0.15);      // floor
      landmarks.emplace_back(ux(rng), 10.0, uz(rng));       // left wall
      landmarks.emplace_back(ux(rng), -8.0, uz(rng));       // right wall
      landmarks.emplace_back(ux(rng), uy(rng), 2.5);        // ceiling
      landmarks.emplace_back(48.0, uy(rng), uz(rng));       // front wall
    }
  }

  static double normal(std::mt19937 & g, double sigma)
  {
    return std::normal_distribution<double>(0.0, sigma)(g);
  }

  ground_vio::ImuSample imu(double t, double rate_hz)
  {
    const double dt = 1.0 / rate_hz;
    const Eigen::Matrix3d R = traj.orientation(t).toRotationMatrix();
    ground_vio::ImuSample s;
    s.t = t;
    s.gyro = traj.bodyRate(t) + bg;
    s.accel = R.transpose() * (traj.acceleration(t) - gravity) + ba;
    const double sg = gyro_noise / std::sqrt(dt);
    const double sa = accel_noise / std::sqrt(dt);
    for (int i = 0; i < 3; ++i) {
      s.gyro(i) += normal(rng_imu, sg);
      s.accel(i) += normal(rng_imu, sa);
    }
    return s;
  }

  // Features seen by the camera at time t. The id of a landmark is its index, so a
  // landmark keeps its id for as long as it stays in view.
  std::vector<ground_vio::FeatureObservation> observe(double t)
  {
    std::vector<ground_vio::FeatureObservation> out;
    const Eigen::Matrix3d R_WI = traj.orientation(t).toRotationMatrix();
    const Eigen::Vector3d p_WI = traj.position(t);
    const Eigen::Matrix3d R_WC = R_WI * q_IC.toRotationMatrix();
    const Eigen::Vector3d p_WC = p_WI + R_WI * p_IC;
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    auto u01 = [&]() {return uni(rng_cam);};
    for (size_t i = 0; i < landmarks.size(); i += 4) {
      const Eigen::Vector3d pc = R_WC.transpose() * (landmarks[i] - p_WC);
      if (pc.z() < 0.5 || pc.z() > 40.0) {
        continue;
      }
      const double u = cam.f * pc.x() / pc.z() + cam.cx;
      const double v = cam.f * pc.y() / pc.z() + cam.cy;
      if (u < 5 || u > cam.width - 5 || v < 5 || v > cam.height - 5) {
        continue;
      }
      double du = normal(rng_cam, pixel_noise), dv = normal(rng_cam, pixel_noise);
      if (u01() < outlier_rate) {
        du += (u01() < 0.5 ? -1.0 : 1.0) * (3.0 + 12.0 * u01());
        dv += (u01() < 0.5 ? -1.0 : 1.0) * (3.0 + 12.0 * u01());
      }
      ground_vio::FeatureObservation o;
      o.id = static_cast<int64_t>(i);
      o.uv = Eigen::Vector2d((u + du - cam.cx) / cam.f, (v + dv - cam.cy) / cam.f);
      out.push_back(o);
    }
    return out;
  }
};

struct RunResult
{
  double final_pos_error = 0.0;
  double final_yaw_error_deg = 0.0;
  double final_ext_rot_error_deg = 0.0;
  double final_ext_pos_error = 0.0;
  double path_length = 0.0;
  double mean_nees = 0.0;
  double max_speed_in_rest = 0.0;
  ground_vio::FilterStats stats;
  Eigen::Vector3d final_position;
};

// Runs the filter on the synthetic data. image_delay is how long after its time an image
// reaches the filter (negative = the image arrives before the IMU data that covers it).
inline RunResult run(
  World & world, ground_vio::FilterParams params, double duration, double image_delay,
  bool verbose)
{
  const double imu_rate = 200.0;
  const double cam_rate = 30.0;
  ground_vio::VioFilter filter(params);
  RunResult res;

  double next_image = 0.0;
  Eigen::Vector3d last_truth = world.traj.position(0.0);
  std::vector<double> nees;
  double next_print = 5.0;
  for (double t = 0.0; t <= duration; t += 1.0 / imu_rate) {
    filter.addImu(world.imu(t, imu_rate));
    while (next_image + image_delay <= t) {
      // Observations are generated once, from the true state at the image time.
      filter.addImage(next_image, world.observe(next_image));
      next_image += 1.0 / cam_rate;
    }
    filter.process();

    if (filter.initialized()) {
      const double te = filter.time();
      const Eigen::Vector3d truth = world.traj.position(te);
      res.path_length += (truth - last_truth).norm();
      last_truth = truth;
      if (te < world.traj.still_s) {
        res.max_speed_in_rest = std::max(res.max_speed_in_rest, filter.velocity().norm());
      }
      if (te > 6.0) {
        const Eigen::Vector3d e = filter.position() - truth;
        const Eigen::Matrix3d Pp = filter.coreCovariance().block<3, 3>(3, 3);
        nees.push_back(e.dot(Pp.ldlt().solve(e)));
      }
      if (verbose && te >= next_print) {
        const Eigen::Vector3d e = filter.position() - truth;
        const Eigen::Quaterniond dq =
          filter.orientation().conjugate() * world.traj.orientation(te);
        std::printf(
          "  t=%5.1f  pos err %.3f m  yaw err %.2f deg  clones %d  used %d  rejected: geometry %d gate %d\n",
          te, e.norm(), std::abs(ground_vio::logQuat(dq).z()) * 180.0 / M_PI,
          filter.numClones(), filter.stats().features_used, filter.stats().rejected_geometry,
          filter.stats().rejected_gate);
        next_print += 5.0;
      }
    }
  }
  const double te = filter.time();
  res.final_position = filter.position();
  res.final_pos_error = (filter.position() - world.traj.position(te)).norm();
  const Eigen::Quaterniond dq = filter.orientation().conjugate() * world.traj.orientation(te);
  res.final_yaw_error_deg = ground_vio::logQuat(dq).norm() * 180.0 / M_PI;
  res.final_ext_rot_error_deg =
    ground_vio::logQuat(filter.extrinsicRotation().conjugate() * world.q_IC).norm() * 180.0 / M_PI;
  res.final_ext_pos_error = (filter.extrinsicPosition() - world.p_IC).norm();
  double s = 0.0;
  for (double v : nees) {
    s += v;
  }
  res.mean_nees = nees.empty() ? 0.0 : s / static_cast<double>(nees.size());
  res.stats = filter.stats();
  return res;
}

inline ground_vio::FilterParams defaultParams(const World & world)
{
  ground_vio::FilterParams p;
  p.gravity = world.gravity;
  p.imu.gyro_noise = 2.0 * world.gyro_noise;
  p.imu.accel_noise = 2.0 * world.accel_noise;
  p.imu.gyro_walk = 1e-5;
  p.imu.accel_walk = 1e-4;
  p.q_imu_cam = world.q_IC;
  p.p_imu_cam = world.p_IC;
  p.pixel_sigma = 1.0 / world.cam.f;
  p.estimate_extrinsics = false;
  return p;
}

}  // namespace sim
