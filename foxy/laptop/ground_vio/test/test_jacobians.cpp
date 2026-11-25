// test_jacobians.cpp
// Compares the analytic Jacobians of the camera projection with numerical derivatives.
#include <cmath>
#include <cstdio>

#include "ground_vio/math_utils.hpp"
#include "ground_vio/vio_filter.hpp"
#include "test_common.hpp"

using namespace ground_vio;

namespace
{
Eigen::Vector2d project(
  const Eigen::Quaterniond & q, const Eigen::Vector3d & p,
  const Eigen::Quaterniond & q_ic, const Eigen::Vector3d & p_ic, const Eigen::Vector3d & pf)
{
  return detail::projectionJacobians(q, p, q_ic, p_ic, pf).h;
}
}  // namespace

int main()
{
  const Eigen::Quaterniond q_WI = expQuat(Eigen::Vector3d(0.3, -0.2, 0.7));
  const Eigen::Vector3d p_WI(1.0, -0.5, 0.2);
  const Eigen::Quaterniond q_IC = expQuat(Eigen::Vector3d(-1.2, 0.1, 0.4));
  const Eigen::Vector3d p_IC(0.08, 0.02, -0.05);
  Eigen::Vector3d pf = p_WI + q_WI * q_IC * Eigen::Vector3d(0.4, -0.3, 4.0) +
    q_WI * p_IC;

  const auto J = detail::projectionJacobians(q_WI, p_WI, q_IC, p_IC, pf);
  const double eps = 1e-6;
  double worst = 0.0;

  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    d(i) = eps;
    auto num = [&](auto f) {
        return ((f(d) - f(-d)) / (2.0 * eps)).eval();
      };
    const Eigen::Vector2d g_feat = num([&](const Eigen::Vector3d & x) {
        return project(q_WI, p_WI, q_IC, p_IC, pf + x);
      });
    const Eigen::Vector2d g_pos = num([&](const Eigen::Vector3d & x) {
        return project(q_WI, p_WI + x, q_IC, p_IC, pf);
      });
    const Eigen::Vector2d g_th = num([&](const Eigen::Vector3d & x) {
        return project(q_WI * expQuat(x), p_WI, q_IC, p_IC, pf);
      });
    const Eigen::Vector2d g_pic = num([&](const Eigen::Vector3d & x) {
        return project(q_WI, p_WI, q_IC, p_IC + x, pf);
      });
    const Eigen::Vector2d g_thic = num([&](const Eigen::Vector3d & x) {
        return project(q_WI, p_WI, q_IC * expQuat(x), p_IC, pf);
      });
    worst = std::max(worst, (g_feat - J.H_feat.col(i)).norm());
    worst = std::max(worst, (g_pos - J.H_pos.col(i)).norm());
    worst = std::max(worst, (g_th - J.H_theta.col(i)).norm());
    worst = std::max(worst, (g_pic - J.H_pos_ic.col(i)).norm());
    worst = std::max(worst, (g_thic - J.H_theta_ic.col(i)).norm());
  }
  std::printf("largest Jacobian difference: %.3e\n", worst);
  CHECK(worst < 1e-6, "analytic and numerical Jacobians differ by %.3e", worst);

  // exp / log round trip and chi-square quantile
  const Eigen::Vector3d w(0.4, -1.1, 0.9);
  CHECK((logQuat(expQuat(w)) - w).norm() < 1e-12, "log/exp round trip");
  CHECK(std::abs(detail::chi2Quantile95(1) - 3.84) < 0.2, "chi2(1)=%f", detail::chi2Quantile95(1));
  CHECK(std::abs(detail::chi2Quantile95(10) - 18.31) < 0.2, "chi2(10)=%f", detail::chi2Quantile95(10));
  CHECK(std::abs(detail::chi2Quantile95(100) - 124.34) < 0.5, "chi2(100)=%f", detail::chi2Quantile95(100));
  return TEST_RESULT();
}
