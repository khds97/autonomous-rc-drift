// vio_filter.hpp
// Monocular visual-inertial odometry as a multi-state constraint Kalman filter (MSCKF).
//
// Frames: W is the world frame (z up, gravity along -z, the origin and heading are the pose
// at start), I is the IMU frame and C is the camera frame (x right, y down, z forward).
// Rotations are Hamilton quaternions, q_WI rotates a vector from I to W. Error states use
// a local perturbation, R_true = R_est * Exp(dtheta).
//
// The filter has no ROS dependency. Feed it IMU samples and the tracked features of each
// image (in normalized coordinates, i.e. already undistorted), then call process().
#pragma once

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ground_vio
{

struct ImuSample
{
  double t = 0.0;                                 // seconds
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s^2 (specific force)
};

// One tracked feature in one image. uv = (x/z, y/z) of the camera-frame ray.
struct FeatureObservation
{
  int64_t id = 0;
  Eigen::Vector2d uv = Eigen::Vector2d::Zero();
};

struct ImuNoise
{
  double gyro_noise = 2e-4;    // rad/s/sqrt(Hz)
  double accel_noise = 2e-3;   // m/s^2/sqrt(Hz)
  double gyro_walk = 1e-5;     // rad/s^2/sqrt(Hz)
  double accel_walk = 1e-4;    // m/s^3/sqrt(Hz)
};

struct FilterParams
{
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  ImuNoise imu;

  // Camera pose in the IMU frame (initial guess if the extrinsics are estimated).
  Eigen::Quaterniond q_imu_cam = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_imu_cam = Eigen::Vector3d::Zero();
  bool estimate_extrinsics = true;
  double extrinsic_rot_sigma = 0.05;   // rad
  double extrinsic_pos_sigma = 0.03;   // m

  double pixel_sigma = 0.002;          // image noise in normalized units (pixels / focal length)
  int max_clones = 11;                 // camera poses kept in the sliding window
  int min_track_length = 4;            // observations needed before a feature is used
  double min_parallax_deg = 1.0;       // smallest ray angle accepted for triangulation
  double min_depth = 0.2;              // m
  double max_depth = 30.0;             // m
  double chi2_scale = 1.0;             // multiplier of the 95 % chi-square gate

  // Static initialization: the vehicle must stand still for init_window_s.
  double init_window_s = 1.0;
  double init_gyro_std = 0.03;         // rad/s
  double init_accel_std = 0.3;         // m/s^2

  // Optional constraints for a car on a flat floor.
  double planar_z_sigma = 0.0;         // m, 0 = off. Keeps the height at its start value.
  bool zupt_enabled = false;           // zero-velocity update while standing still
  double zupt_window_s = 0.3;
  double zupt_gyro_std = 0.02;         // rad/s
  double zupt_accel_std = 0.15;        // m/s^2
  double zupt_accel_tol = 0.3;         // m/s^2, mean specific force must match gravity
  double zupt_sigma = 0.02;            // m/s
  // An IMU alone cannot tell rest from constant velocity, so the tracked features must
  // also stand still: median motion over zupt_window_s in normalized units (1.2 px at f=400).
  double zupt_max_disparity = 0.003;
  int zupt_min_features = 15;
};

struct FilterStats
{
  int frames = 0;
  int features_used = 0;
  int rejected_geometry = 0;   // no usable triangulation (parallax, depth, behind the camera)
  int rejected_gate = 0;       // failed the chi-square gate
  int zupt_updates = 0;
};

class VioFilter
{
public:
  explicit VioFilter(const FilterParams & params);

  void addImu(const ImuSample & sample);
  // t is the image time on the IMU clock.
  void addImage(double t, const std::vector<FeatureObservation> & observations);
  // Handles all queued data that can be handled. An image waits until the IMU data covers its time.
  void process();

  bool initialized() const {return initialized_;}
  double time() const {return t_;}
  const Eigen::Quaterniond & orientation() const {return q_;}
  const Eigen::Vector3d & position() const {return p_;}
  const Eigen::Vector3d & velocity() const {return v_;}
  const Eigen::Vector3d & gyroBias() const {return bg_;}
  const Eigen::Vector3d & accelBias() const {return ba_;}
  const Eigen::Quaterniond & extrinsicRotation() const {return q_ic_;}
  const Eigen::Vector3d & extrinsicPosition() const {return p_ic_;}
  int numClones() const {return static_cast<int>(clones_.size());}
  // Covariance of [theta, p, v, bg, ba, theta_ic, p_ic].
  Eigen::MatrixXd coreCovariance() const {return P_.topLeftCorner(21, 21);}
  const FilterStats & stats() const {return stats_;}

private:
  struct Clone
  {
    int64_t id;
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
  };
  struct Observation
  {
    int64_t clone_id;
    Eigen::Vector2d uv;
  };
  struct Track
  {
    std::vector<Observation> obs;
  };
  struct ImageFrame
  {
    double t;
    std::vector<FeatureObservation> obs;
  };

  bool tryInitialize();
  void propagateTo(double target);
  void propagateStep(const Eigen::Vector3d & gyro, const Eigen::Vector3d & accel, double dt);
  void processFrame(const ImageFrame & frame);
  void augmentClone();
  void removeOldestClone();
  void visualUpdate(const std::vector<int64_t> & ids);
  enum class FeatureResult {kOk, kGeometry, kGate};
  FeatureResult featureResidual(
    const Track & track, Eigen::MatrixXd & H_o, Eigen::VectorXd & r_o) const;
  void applyUpdate(const Eigen::MatrixXd & H, const Eigen::VectorXd & r, double sigma2);
  void applyCorrection(const Eigen::VectorXd & dx);
  void planarUpdate();
  void zeroVelocityUpdate();

  FilterParams params_;
  bool initialized_ = false;
  double t_ = 0.0;

  Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_ic_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_ic_ = Eigen::Vector3d::Zero();
  double start_height_ = 0.0;

  std::deque<Clone> clones_;
  int64_t next_clone_id_ = 0;
  Eigen::MatrixXd P_;

  std::unordered_map<int64_t, Track> tracks_;
  std::deque<ImuSample> imu_;
  std::deque<ImuSample> recent_;   // propagated samples, for the stationary check
  std::deque<ImageFrame> images_;
  struct FeatureSnapshot
  {
    double t;
    std::unordered_map<int64_t, Eigen::Vector2d> uv;
  };
  std::deque<FeatureSnapshot> feature_history_;
  double last_disparity_ = 1e9;   // median feature motion over the last zupt_window_s
  int last_disparity_count_ = 0;
  FilterStats stats_;
};

namespace detail
{
// Projection of a world point into a camera, and the Jacobians of the normalized
// coordinates with respect to the error states. Exposed for the unit tests.
struct ProjectionJacobians
{
  Eigen::Vector3d p_cam;                 // point in the camera frame
  Eigen::Vector2d h;                     // normalized coordinates
  Eigen::Matrix<double, 2, 3> H_feat;    // world point
  Eigen::Matrix<double, 2, 3> H_theta;   // IMU orientation of the clone
  Eigen::Matrix<double, 2, 3> H_pos;     // IMU position of the clone
  Eigen::Matrix<double, 2, 3> H_theta_ic;
  Eigen::Matrix<double, 2, 3> H_pos_ic;
};

ProjectionJacobians projectionJacobians(
  const Eigen::Quaterniond & q_WI, const Eigen::Vector3d & p_WI,
  const Eigen::Quaterniond & q_IC, const Eigen::Vector3d & p_IC,
  const Eigen::Vector3d & p_f_W);

// 95 % quantile of the chi-square distribution (Wilson-Hilferty approximation).
double chi2Quantile95(int dof);
}  // namespace detail

}  // namespace ground_vio
