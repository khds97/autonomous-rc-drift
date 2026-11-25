// test_filter_sim.cpp
// Runs the filter on a synthetic drive with 1.5 % outlier tracks and checks the error.
#include <cstdio>

#include "sim_world.hpp"
#include "test_common.hpp"

using namespace ground_vio;

int main(int argc, char **)
{
  const bool verbose = argc > 1;

  // 1. Known extrinsics, 3D motion.
  {
    sim::World world;
    auto params = sim::defaultParams(world);
    auto res = sim::run(world, params, 40.0, 0.02, verbose);
    std::printf(
      "[known extrinsics] path %.1f m  final position error %.3f m  yaw error %.2f deg  "
      "NEES %.2f  features used %d, rejected by geometry %d, by gate %d\n",
      res.path_length, res.final_pos_error, res.final_yaw_error_deg, res.mean_nees,
      res.stats.features_used, res.stats.rejected_geometry, res.stats.rejected_gate);
    CHECK(res.path_length > 20.0, "path too short: %.1f", res.path_length);
    CHECK(res.final_pos_error < 0.5, "position error %.3f m", res.final_pos_error);
    CHECK(res.final_yaw_error_deg < 3.0, "attitude error %.2f deg", res.final_yaw_error_deg);
    CHECK(res.stats.features_used > 500, "too few features used: %d", res.stats.features_used);
  }

  // 2. Wrong extrinsic guess that the filter has to correct.
  {
    sim::World world;
    auto params = sim::defaultParams(world);
    params.estimate_extrinsics = true;
    params.q_imu_cam = world.q_IC * expQuat(Eigen::Vector3d(0.03, -0.025, 0.02));   // about 2.3 deg
    params.p_imu_cam = world.p_IC + Eigen::Vector3d(0.02, -0.015, 0.01);
    const double rot0 = logQuat(params.q_imu_cam.conjugate() * world.q_IC).norm() * 180.0 / M_PI;
    const double pos0 = (params.p_imu_cam - world.p_IC).norm();
    auto res = sim::run(world, params, 40.0, 0.02, verbose);
    std::printf(
      "[extrinsic calibration] position error %.3f m  yaw error %.2f deg  extrinsic rotation "
      "error %.2f -> %.2f deg  extrinsic position error %.3f -> %.3f m\n",
      res.final_pos_error, res.final_yaw_error_deg, rot0, res.final_ext_rot_error_deg, pos0,
      res.final_ext_pos_error);
    CHECK(res.final_pos_error < 0.6, "position error %.3f m", res.final_pos_error);
    CHECK(res.final_ext_rot_error_deg < rot0, "rotation extrinsic not improved");
    CHECK(res.final_ext_rot_error_deg < 1.0, "rotation extrinsic %.2f deg", res.final_ext_rot_error_deg);
  }

  // 3. Flat floor: planar constraint and zero-velocity update while standing still.
  {
    sim::World world;
    world.traj.planar = true;
    auto params = sim::defaultParams(world);
    params.planar_z_sigma = 0.02;
    params.zupt_enabled = true;
    auto res = sim::run(world, params, 40.0, 0.02, verbose);
    std::printf(
      "[flat floor] position error %.3f m  yaw error %.2f deg  zero-velocity updates %d  "
      "speed while still %.4f m/s\n",
      res.final_pos_error, res.final_yaw_error_deg, res.stats.zupt_updates,
      res.max_speed_in_rest);
    CHECK(res.final_pos_error < 0.8, "position error %.3f m", res.final_pos_error);
    CHECK(res.stats.zupt_updates > 0, "no zero-velocity update was applied");
    CHECK(res.max_speed_in_rest < 0.05, "speed while still %.4f", res.max_speed_in_rest);
  }

  // 4. An image that reaches the filter before the IMU data covering it waits and gives
  //    the same result as an image that arrives late.
  {
    sim::World w1, w2;
    auto p1 = sim::defaultParams(w1);
    auto p2 = sim::defaultParams(w2);
    auto late = sim::run(w1, p1, 20.0, 0.02, false);
    auto early = sim::run(w2, p2, 20.0, -0.01, false);
    const double diff = (late.final_position - early.final_position).norm();
    std::printf("[image order] difference between late and early images: %.2e m\n", diff);
    CHECK(diff < 1e-6, "late and early images give different results (%.2e)", diff);
  }
  return TEST_RESULT();
}
