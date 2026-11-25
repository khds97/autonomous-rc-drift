// vio_filter.cpp
#include "ground_vio/vio_filter.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "ground_vio/math_utils.hpp"

namespace ground_vio
{

namespace
{
// Layout of the error state: the core block followed by 6 values per camera clone.
constexpr int kTheta = 0;
constexpr int kPos = 3;
constexpr int kVel = 6;
constexpr int kBg = 9;
constexpr int kBa = 12;
constexpr int kExtRot = 15;
constexpr int kExtPos = 18;
constexpr int kCore = 21;
constexpr int kCloneDim = 6;
constexpr size_t kMaxImuQueue = 4000;
constexpr size_t kMaxImageQueue = 200;

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

ImuSample lerpImu(const ImuSample & a, const ImuSample & b, double t)
{
  const double span = b.t - a.t;
  const double w = span > 0.0 ? std::min(1.0, std::max(0.0, (t - a.t) / span)) : 0.0;
  ImuSample s;
  s.t = t;
  s.gyro = (1.0 - w) * a.gyro + w * b.gyro;
  s.accel = (1.0 - w) * a.accel + w * b.accel;
  return s;
}

struct Stat3
{
  Vector3d mean = Vector3d::Zero();
  Vector3d stddev = Vector3d::Zero();
};

template<typename Container, typename Getter>
Stat3 statistics(const Container & c, Getter get)
{
  Stat3 s;
  if (c.empty()) {
    return s;
  }
  for (const auto & x : c) {
    s.mean += get(x);
  }
  s.mean /= static_cast<double>(c.size());
  Vector3d var = Vector3d::Zero();
  for (const auto & x : c) {
    const Vector3d d = get(x) - s.mean;
    var += d.cwiseProduct(d);
  }
  var /= static_cast<double>(c.size());
  s.stddev = var.cwiseSqrt();
  return s;
}

// Camera poses used to triangulate one feature.
struct CameraView
{
  Matrix3d R_WC;
  Vector3d p_WC;
  Vector2d uv;
};

// Triangulates a world point from several views: linear midpoint solution followed by a
// few damped Gauss-Newton steps on the reprojection error. Returns false if the geometry is
// weak (no parallax, point behind a camera or outside the depth range).
bool triangulate(
  const std::vector<CameraView> & views, const FilterParams & params, Vector3d * p_f)
{
  Matrix3d A = Matrix3d::Zero();
  Vector3d b = Vector3d::Zero();
  for (const auto & v : views) {
    Vector3d d = v.R_WC * Vector3d(v.uv.x(), v.uv.y(), 1.0);
    d.normalize();
    const Matrix3d M = Matrix3d::Identity() - d * d.transpose();
    A += M;
    b += M * v.p_WC;
  }
  Eigen::SelfAdjointEigenSolver<Matrix3d> es(A);
  if (es.eigenvalues()(0) < 1e-9 * es.eigenvalues()(2)) {
    return false;
  }
  Vector3d x = A.ldlt().solve(b);

  double lambda = 1e-3;
  auto cost = [&](const Vector3d & pt, bool * ok) {
      double c = 0.0;
      *ok = true;
      for (const auto & v : views) {
        const Vector3d pc = v.R_WC.transpose() * (pt - v.p_WC);
        if (pc.z() < params.min_depth * 0.5) {
          *ok = false;
          return 0.0;
        }
        c += (v.uv - pc.head<2>() / pc.z()).squaredNorm();
      }
      return c;
    };
  bool ok = true;
  double best = cost(x, &ok);
  if (!ok) {
    return false;
  }
  for (int it = 0; it < 10; ++it) {
    Matrix3d JtJ = Matrix3d::Zero();
    Vector3d Jtr = Vector3d::Zero();
    for (const auto & v : views) {
      const Matrix3d Rt = v.R_WC.transpose();
      const Vector3d pc = Rt * (x - v.p_WC);
      const double iz = 1.0 / pc.z();
      Eigen::Matrix<double, 2, 3> Jh;
      Jh << iz, 0.0, -pc.x() * iz * iz,
        0.0, iz, -pc.y() * iz * iz;
      const Eigen::Matrix<double, 2, 3> J = Jh * Rt;
      const Vector2d r = v.uv - pc.head<2>() * iz;
      JtJ += J.transpose() * J;
      Jtr += J.transpose() * r;
    }
    Matrix3d D = JtJ;
    D.diagonal() *= (1.0 + lambda);
    const Vector3d dx = D.ldlt().solve(Jtr);
    const Vector3d candidate = x + dx;
    bool cand_ok = true;
    const double c = cost(candidate, &cand_ok);
    if (cand_ok && c < best) {
      x = candidate;
      best = c;
      lambda = std::max(1e-7, lambda * 0.3);
      if (dx.norm() < 1e-6) {
        break;
      }
    } else {
      lambda *= 10.0;
    }
  }

  // Depth range in every view and enough parallax between the first view and the others.
  double max_angle = 0.0;
  const Vector3d r0 = (views.front().p_WC - x).normalized();
  for (const auto & v : views) {
    const Vector3d pc = v.R_WC.transpose() * (x - v.p_WC);
    if (pc.z() < params.min_depth || pc.z() > params.max_depth) {
      return false;
    }
    const Vector3d ri = (v.p_WC - x).normalized();
    max_angle = std::max(max_angle, std::acos(std::min(1.0, std::max(-1.0, r0.dot(ri)))));
  }
  if (max_angle * 180.0 / M_PI < params.min_parallax_deg) {
    return false;
  }
  *p_f = x;
  return true;
}

}  // namespace

namespace detail
{

double chi2Quantile95(int dof)
{
  const double k = static_cast<double>(std::max(1, dof));
  const double a = 2.0 / (9.0 * k);
  const double t = 1.0 - a + 1.6449 * std::sqrt(a);
  return k * t * t * t;
}

ProjectionJacobians projectionJacobians(
  const Eigen::Quaterniond & q_WI, const Eigen::Vector3d & p_WI,
  const Eigen::Quaterniond & q_IC, const Eigen::Vector3d & p_IC,
  const Eigen::Vector3d & p_f_W)
{
  const Matrix3d R_WI = q_WI.toRotationMatrix();
  const Matrix3d R_IC = q_IC.toRotationMatrix();
  const Matrix3d R_WC = R_WI * R_IC;
  const Vector3d p_WC = p_WI + R_WI * p_IC;
  const Vector3d p_f_I = R_WI.transpose() * (p_f_W - p_WI);

  ProjectionJacobians out;
  out.p_cam = R_WC.transpose() * (p_f_W - p_WC);
  const double iz = 1.0 / out.p_cam.z();
  out.h = out.p_cam.head<2>() * iz;
  Eigen::Matrix<double, 2, 3> Jh;
  Jh << iz, 0.0, -out.p_cam.x() * iz * iz,
    0.0, iz, -out.p_cam.y() * iz * iz;

  out.H_feat = Jh * R_WC.transpose();
  out.H_pos = -Jh * R_WC.transpose();
  out.H_theta = Jh * R_IC.transpose() * skew(p_f_I);
  out.H_theta_ic = Jh * skew(out.p_cam);
  out.H_pos_ic = -Jh * R_IC.transpose();
  return out;
}

}  // namespace detail

VioFilter::VioFilter(const FilterParams & params)
: params_(params)
{
}

void VioFilter::addImu(const ImuSample & sample)
{
  if (!imu_.empty() && sample.t <= imu_.back().t) {
    return;
  }
  imu_.push_back(sample);
  if (!initialized_) {
    // Before the start only the newest window is needed.
    while (imu_.back().t - imu_.front().t > 2.0 * params_.init_window_s) {
      imu_.pop_front();
    }
  } else if (imu_.size() > kMaxImuQueue) {
    imu_.pop_front();
  }
}

void VioFilter::addImage(double t, const std::vector<FeatureObservation> & observations)
{
  images_.push_back({t, observations});
  if (images_.size() > kMaxImageQueue) {
    images_.pop_front();
  }
}

bool VioFilter::tryInitialize()
{
  if (imu_.size() < 10) {
    return false;
  }
  const double t_end = imu_.back().t;
  std::vector<const ImuSample *> window;
  for (const auto & s : imu_) {
    if (s.t >= t_end - params_.init_window_s) {
      window.push_back(&s);
    }
  }
  if (window.size() < 10 || t_end - window.front()->t < 0.9 * params_.init_window_s) {
    return false;
  }
  const Stat3 g = statistics(window, [](const ImuSample * s) {return s->gyro;});
  const Stat3 a = statistics(window, [](const ImuSample * s) {return s->accel;});
  if (g.stddev.maxCoeff() > params_.init_gyro_std ||
    a.stddev.maxCoeff() > params_.init_accel_std)
  {
    return false;
  }
  const double g_norm = params_.gravity.norm();
  if (std::abs(a.mean.norm() - g_norm) > 0.15 * g_norm) {
    return false;
  }

  // At rest the accelerometer measures -gravity, so it points up in the body frame.
  q_ = Eigen::Quaterniond::FromTwoVectors(a.mean, -params_.gravity.normalized());
  p_.setZero();
  v_.setZero();
  bg_ = g.mean;
  ba_.setZero();
  q_ic_ = params_.q_imu_cam.normalized();
  p_ic_ = params_.p_imu_cam;
  start_height_ = 0.0;

  P_ = MatrixXd::Zero(kCore, kCore);
  P_.block<2, 2>(kTheta, kTheta) = Eigen::Matrix2d::Identity() * 0.02 * 0.02;
  P_(kTheta + 2, kTheta + 2) = 1e-6;   // heading is the reference of the world frame
  P_.block<3, 3>(kPos, kPos) = Matrix3d::Identity() * 1e-6;
  P_.block<3, 3>(kVel, kVel) = Matrix3d::Identity() * 1e-4;
  P_.block<3, 3>(kBg, kBg) = Matrix3d::Identity() * 1e-6;
  P_.block<3, 3>(kBa, kBa) = Matrix3d::Identity() * 1e-2;
  const double er = params_.estimate_extrinsics ? params_.extrinsic_rot_sigma : 1e-6;
  const double ep = params_.estimate_extrinsics ? params_.extrinsic_pos_sigma : 1e-6;
  P_.block<3, 3>(kExtRot, kExtRot) = Matrix3d::Identity() * er * er;
  P_.block<3, 3>(kExtPos, kExtPos) = Matrix3d::Identity() * ep * ep;

  t_ = imu_.back().t;
  while (imu_.size() > 1) {
    imu_.pop_front();
  }
  initialized_ = true;
  return true;
}

void VioFilter::process()
{
  if (!initialized_ && !tryInitialize()) {
    return;
  }
  while (!images_.empty()) {
    const ImageFrame & frame = images_.front();
    if (frame.t < t_) {
      images_.pop_front();   // older than the state, cannot be used
      continue;
    }
    if (imu_.empty() || imu_.back().t < frame.t) {
      break;   // IMU data does not reach the image yet
    }
    propagateTo(frame.t);
    processFrame(frame);
    images_.pop_front();
  }
}

void VioFilter::propagateTo(double target)
{
  while (t_ < target - 1e-12) {
    // Find the IMU interval that contains the current state time.
    size_t j = 0;
    while (j + 1 < imu_.size() && imu_[j + 1].t <= t_) {
      ++j;
    }
    if (j + 1 >= imu_.size() || imu_[j].t > t_ + 1e-9) {
      break;
    }
    const double t_end = std::min(target, imu_[j + 1].t);
    const ImuSample m0 = lerpImu(imu_[j], imu_[j + 1], t_);
    const ImuSample m1 = lerpImu(imu_[j], imu_[j + 1], t_end);
    const double dt = t_end - t_;
    if (dt > 0.0) {
      propagateStep(0.5 * (m0.gyro + m1.gyro), 0.5 * (m0.accel + m1.accel), dt);
      ImuSample r = m1;
      recent_.push_back(r);
      while (recent_.size() > 1 && recent_.back().t - recent_.front().t > params_.zupt_window_s) {
        recent_.pop_front();
      }
    }
    t_ = t_end;
    while (imu_.size() > 1 && imu_[1].t <= t_) {
      imu_.pop_front();
    }
  }
}

void VioFilter::propagateStep(const Vector3d & gyro, const Vector3d & accel, double dt)
{
  const Vector3d w = gyro - bg_;
  const Vector3d a_body = accel - ba_;
  const Matrix3d R = q_.toRotationMatrix();
  const Matrix3d dR = expMat(w * dt);

  // Error-state transition of the 15 IMU states (the extrinsics are constant).
  MatrixXd F = MatrixXd::Identity(kCore, kCore);
  F.block<3, 3>(kTheta, kTheta) = dR.transpose();
  F.block<3, 3>(kTheta, kBg) = -Matrix3d::Identity() * dt;
  F.block<3, 3>(kPos, kVel) = Matrix3d::Identity() * dt;
  F.block<3, 3>(kPos, kTheta) = -0.5 * R * skew(a_body) * dt * dt;
  F.block<3, 3>(kPos, kBa) = -0.5 * R * dt * dt;
  F.block<3, 3>(kVel, kTheta) = -R * skew(a_body) * dt;
  F.block<3, 3>(kVel, kBa) = -R * dt;

  const ImuNoise & n = params_.imu;
  P_.topRows(kCore) = F * P_.topRows(kCore);
  P_.leftCols(kCore) = P_.leftCols(kCore) * F.transpose();
  P_.block<3, 3>(kTheta, kTheta) += Matrix3d::Identity() * n.gyro_noise * n.gyro_noise * dt;
  P_.block<3, 3>(kVel, kVel) += Matrix3d::Identity() * n.accel_noise * n.accel_noise * dt;
  P_.block<3, 3>(kBg, kBg) += Matrix3d::Identity() * n.gyro_walk * n.gyro_walk * dt;
  P_.block<3, 3>(kBa, kBa) += Matrix3d::Identity() * n.accel_walk * n.accel_walk * dt;

  const Vector3d acc_world = R * a_body + params_.gravity;
  p_ += v_ * dt + 0.5 * acc_world * dt * dt;
  v_ += acc_world * dt;
  q_ = (q_ * expQuat(w * dt)).normalized();
}

void VioFilter::augmentClone()
{
  const int n = static_cast<int>(P_.rows());
  MatrixXd P_new = MatrixXd::Zero(n + kCloneDim, n + kCloneDim);
  P_new.topLeftCorner(n, n) = P_;
  P_new.block(n, 0, kCloneDim, n) = P_.topRows(kCloneDim);
  P_new.block(0, n, n, kCloneDim) = P_.leftCols(kCloneDim);
  P_new.block(n, n, kCloneDim, kCloneDim) = P_.topLeftCorner(kCloneDim, kCloneDim);
  P_ = P_new;
  clones_.push_back({next_clone_id_++, q_, p_});
}

void VioFilter::removeOldestClone()
{
  const int n = static_cast<int>(P_.rows());
  const int start = kCore;
  const int rest = n - start - kCloneDim;
  MatrixXd P_new(n - kCloneDim, n - kCloneDim);
  P_new.topLeftCorner(start, start) = P_.topLeftCorner(start, start);
  P_new.topRightCorner(start, rest) = P_.block(0, start + kCloneDim, start, rest);
  P_new.bottomLeftCorner(rest, start) = P_.block(start + kCloneDim, 0, rest, start);
  P_new.bottomRightCorner(rest, rest) =
    P_.block(start + kCloneDim, start + kCloneDim, rest, rest);
  P_ = P_new;
  clones_.pop_front();
}

void VioFilter::processFrame(const ImageFrame & frame)
{
  ++stats_.frames;

  // Median motion over about zupt_window_s of the features that were also seen back then.
  {
    FeatureSnapshot now;
    now.t = frame.t;
    for (const auto & o : frame.obs) {
      now.uv[o.id] = o.uv;
    }
    last_disparity_ = 1e9;
    last_disparity_count_ = 0;
    if (!feature_history_.empty() &&
      frame.t - feature_history_.front().t >= 0.8 * params_.zupt_window_s)
    {
      std::vector<double> d;
      for (const auto & kv : now.uv) {
        const auto it = feature_history_.front().uv.find(kv.first);
        if (it != feature_history_.front().uv.end()) {
          d.push_back((kv.second - it->second).norm());
        }
      }
      last_disparity_count_ = static_cast<int>(d.size());
      if (!d.empty()) {
        std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
        last_disparity_ = d[d.size() / 2];
      }
    }
    feature_history_.push_back(std::move(now));
    while (feature_history_.size() > 1 &&
      frame.t - feature_history_[1].t >= params_.zupt_window_s)
    {
      feature_history_.pop_front();
    }
  }

  augmentClone();
  const int64_t clone_id = clones_.back().id;

  std::unordered_set<int64_t> present;
  for (const auto & o : frame.obs) {
    if (!present.insert(o.id).second) {
      continue;   // the same id twice in one image
    }
    tracks_[o.id].obs.push_back({clone_id, o.uv});
  }

  std::vector<int64_t> to_use;
  std::vector<int64_t> to_drop;
  for (const auto & kv : tracks_) {
    if (present.count(kv.first) == 0) {
      // The track ended.
      if (static_cast<int>(kv.second.obs.size()) >= params_.min_track_length) {
        to_use.push_back(kv.first);
      } else {
        to_drop.push_back(kv.first);
      }
    }
  }

  // The oldest clone leaves the window, so every feature seen from it is used now.
  const bool marginalize = static_cast<int>(clones_.size()) > params_.max_clones;
  if (marginalize) {
    const int64_t oldest = clones_.front().id;
    for (const auto & kv : tracks_) {
      if (present.count(kv.first) != 0 && kv.second.obs.front().clone_id == oldest) {
        if (static_cast<int>(kv.second.obs.size()) >= params_.min_track_length) {
          to_use.push_back(kv.first);
        } else {
          to_drop.push_back(kv.first);
        }
      }
    }
  }

  visualUpdate(to_use);
  for (int64_t id : to_use) {
    tracks_.erase(id);
  }
  for (int64_t id : to_drop) {
    tracks_.erase(id);
  }
  if (marginalize) {
    removeOldestClone();
  }

  if (params_.planar_z_sigma > 0.0) {
    planarUpdate();
  }
  if (params_.zupt_enabled) {
    zeroVelocityUpdate();
  }
}

VioFilter::FeatureResult VioFilter::featureResidual(
  const Track & track, MatrixXd & H_o, VectorXd & r_o) const
{
  const int m = static_cast<int>(track.obs.size());
  if (m < 3) {
    return FeatureResult::kGeometry;
  }
  const int n = static_cast<int>(P_.rows());
  const int64_t first_id = clones_.front().id;
  const Matrix3d R_IC = q_ic_.toRotationMatrix();

  std::vector<CameraView> views(m);
  std::vector<int> index(m);
  for (int i = 0; i < m; ++i) {
    const int64_t k = track.obs[i].clone_id - first_id;
    if (k < 0 || k >= static_cast<int64_t>(clones_.size())) {
      return FeatureResult::kGeometry;
    }
    index[i] = static_cast<int>(k);
    const Clone & c = clones_[k];
    const Matrix3d R_WI = c.q.toRotationMatrix();
    views[i].R_WC = R_WI * R_IC;
    views[i].p_WC = c.p + R_WI * p_ic_;
    views[i].uv = track.obs[i].uv;
  }

  Vector3d p_f;
  if (!triangulate(views, params_, &p_f)) {
    return FeatureResult::kGeometry;
  }

  VectorXd r(2 * m);
  MatrixXd H_x = MatrixXd::Zero(2 * m, n);
  MatrixXd H_f(2 * m, 3);
  for (int i = 0; i < m; ++i) {
    const Clone & c = clones_[index[i]];
    const detail::ProjectionJacobians J =
      detail::projectionJacobians(c.q, c.p, q_ic_, p_ic_, p_f);
    if (J.p_cam.z() < params_.min_depth * 0.5) {
      return FeatureResult::kGeometry;
    }
    r.segment<2>(2 * i) = track.obs[i].uv - J.h;
    H_f.block<2, 3>(2 * i, 0) = J.H_feat;
    const int col = kCore + kCloneDim * index[i];
    H_x.block<2, 3>(2 * i, col) = J.H_theta;
    H_x.block<2, 3>(2 * i, col + 3) = J.H_pos;
    if (params_.estimate_extrinsics) {
      H_x.block<2, 3>(2 * i, kExtRot) = J.H_theta_ic;
      H_x.block<2, 3>(2 * i, kExtPos) = J.H_pos_ic;
    }
  }

  // Remove the feature position from the problem with the left null space of H_f.
  Eigen::JacobiSVD<MatrixXd> svd(H_f, Eigen::ComputeFullU);
  const MatrixXd A = svd.matrixU().rightCols(2 * m - 3);
  r_o = A.transpose() * r;
  H_o = A.transpose() * H_x;

  // Chi-square gate against the current covariance.
  const double s2 = params_.pixel_sigma * params_.pixel_sigma;
  MatrixXd S = H_o * P_ * H_o.transpose();
  S.diagonal().array() += s2;
  const double gamma = r_o.dot(S.ldlt().solve(r_o));
  const bool pass =
    gamma <= detail::chi2Quantile95(static_cast<int>(r_o.size())) * params_.chi2_scale;
  return pass ? FeatureResult::kOk : FeatureResult::kGate;
}

void VioFilter::visualUpdate(const std::vector<int64_t> & ids)
{
  std::vector<MatrixXd> Hs;
  std::vector<VectorXd> rs;
  int rows = 0;
  for (int64_t id : ids) {
    MatrixXd H_o;
    VectorXd r_o;
    const FeatureResult result = featureResidual(tracks_.at(id), H_o, r_o);
    if (result == FeatureResult::kOk) {
      rows += static_cast<int>(r_o.size());
      Hs.push_back(std::move(H_o));
      rs.push_back(std::move(r_o));
      ++stats_.features_used;
    } else if (result == FeatureResult::kGeometry) {
      ++stats_.rejected_geometry;
    } else {
      ++stats_.rejected_gate;
    }
  }
  if (rows == 0) {
    return;
  }
  const int n = static_cast<int>(P_.rows());
  MatrixXd H(rows, n);
  VectorXd r(rows);
  int row = 0;
  for (size_t i = 0; i < Hs.size(); ++i) {
    H.middleRows(row, Hs[i].rows()) = Hs[i];
    r.segment(row, rs[i].size()) = rs[i];
    row += static_cast<int>(rs[i].size());
  }

  if (rows > n) {
    // Orthogonal compression keeps the update cost bounded by the state size.
    MatrixXd aug(rows, n + 1);
    aug << H, r;
    Eigen::HouseholderQR<MatrixXd> qr(aug);
    const MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    H = R.topLeftCorner(n, n);
    r = R.block(0, n, n, 1);
  }
  applyUpdate(H, r, params_.pixel_sigma * params_.pixel_sigma);
}

void VioFilter::applyUpdate(const MatrixXd & H, const VectorXd & r, double sigma2)
{
  const MatrixXd PHt = P_ * H.transpose();
  MatrixXd S = H * PHt;
  S.diagonal().array() += sigma2;
  const Eigen::LDLT<MatrixXd> ldlt(S);
  const MatrixXd K = ldlt.solve(PHt.transpose()).transpose();
  const VectorXd dx = K * r;

  const MatrixXd I_KH = MatrixXd::Identity(P_.rows(), P_.cols()) - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + sigma2 * K * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
  applyCorrection(dx);
}

void VioFilter::applyCorrection(const VectorXd & dx)
{
  q_ = (q_ * expQuat(dx.segment<3>(kTheta))).normalized();
  p_ += dx.segment<3>(kPos);
  v_ += dx.segment<3>(kVel);
  bg_ += dx.segment<3>(kBg);
  ba_ += dx.segment<3>(kBa);
  q_ic_ = (q_ic_ * expQuat(dx.segment<3>(kExtRot))).normalized();
  p_ic_ += dx.segment<3>(kExtPos);
  for (size_t k = 0; k < clones_.size(); ++k) {
    const int base = kCore + kCloneDim * static_cast<int>(k);
    clones_[k].q = (clones_[k].q * expQuat(dx.segment<3>(base))).normalized();
    clones_[k].p += dx.segment<3>(base + 3);
  }
}

void VioFilter::planarUpdate()
{
  const int n = static_cast<int>(P_.rows());
  MatrixXd H = MatrixXd::Zero(1, n);
  H(0, kPos + 2) = 1.0;
  VectorXd r(1);
  r(0) = start_height_ - p_.z();
  applyUpdate(H, r, params_.planar_z_sigma * params_.planar_z_sigma);
}

void VioFilter::zeroVelocityUpdate()
{
  if (recent_.size() < 10 ||
    recent_.back().t - recent_.front().t < 0.8 * params_.zupt_window_s)
  {
    return;
  }
  if (last_disparity_count_ < params_.zupt_min_features ||
    last_disparity_ > params_.zupt_max_disparity)
  {
    return;   // the image does not confirm that the vehicle stands still
  }
  const Stat3 g = statistics(recent_, [](const ImuSample & s) {return s.gyro;});
  const Stat3 a = statistics(recent_, [](const ImuSample & s) {return s.accel;});
  // At rest the accelerometer shows only gravity. A steady acceleration has a small spread
  // too, so the mean has to match the expected gravity reading as well.
  const Vector3d rest_reading = q_.toRotationMatrix().transpose() * (-params_.gravity) + ba_;
  if (g.stddev.maxCoeff() > params_.zupt_gyro_std ||
    a.stddev.maxCoeff() > params_.zupt_accel_std ||
    (g.mean - bg_).norm() > 0.05 ||
    (a.mean - rest_reading).norm() > params_.zupt_accel_tol)
  {
    return;
  }
  const int n = static_cast<int>(P_.rows());
  MatrixXd H = MatrixXd::Zero(3, n);
  H.block<3, 3>(0, kVel) = Matrix3d::Identity();
  applyUpdate(H, -v_, params_.zupt_sigma * params_.zupt_sigma);
  ++stats_.zupt_updates;
}

}  // namespace ground_vio
