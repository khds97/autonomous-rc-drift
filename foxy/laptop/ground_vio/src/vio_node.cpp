// vio_node.cpp
// ROS 2 node around the VIO filter: fixed camera + IMU in, nav_msgs/Odometry out.
//
// Both inputs are stamped by the Raspberry Pi on its own clock, so the filter works on those
// stamps only and never on the laptop clock. camera_time_offset_s moves the image stamps onto
// the IMU time axis once the camera-to-IMU offset has been measured.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "ground_vio/feature_tracker.hpp"
#include "ground_vio/vio_filter.hpp"

namespace ground_vio
{

class VioNode : public rclcpp::Node
{
public:
  VioNode()
  : Node("ground_vio")
  {
    declareParameters();
    if (!get_parameter("camera.calibrated").as_bool()) {
      RCLCPP_FATAL(
        get_logger(),
        "camera.calibrated is false. Calibrate the camera (scripts/calibrate_intrinsics.py), "
        "copy the result into config/ground_vio.yaml and set camera.calibrated: true.");
      throw std::runtime_error("camera intrinsics are not calibrated");
    }

    FilterParams fp = readFilterParams();
    filter_ = std::make_unique<VioFilter>(fp);

    const double fx = get_parameter("camera.fx").as_double();
    const double fy = get_parameter("camera.fy").as_double();
    const double cx = get_parameter("camera.cx").as_double();
    const double cy = get_parameter("camera.cy").as_double();
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    const std::vector<double> d = get_parameter("camera.distortion").as_double_array();
    cv::Mat dist(static_cast<int>(d.size()), 1, CV_64F);
    for (size_t i = 0; i < d.size(); ++i) {
      dist.at<double>(static_cast<int>(i)) = d[i];
    }

    TrackerParams tp;
    tp.max_features = static_cast<int>(get_parameter("tracker.max_features").as_int());
    tp.min_distance = get_parameter("tracker.min_distance_px").as_double();
    tracker_ = std::make_unique<FeatureTracker>(K, dist, tp);

    camera_time_offset_ = get_parameter("camera_time_offset_s").as_double();
    frame_id_ = get_parameter("world_frame_id").as_string();
    child_frame_id_ = get_parameter("imu_frame_id").as_string();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&VioNode::onImu, this, std::placeholders::_1));
    if (get_parameter("image_compressed").as_bool()) {
      compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        get_parameter("image_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&VioNode::onCompressed, this, std::placeholders::_1));
    } else {
      raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("image_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&VioNode::onRaw, this, std::placeholders::_1));
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(), 10);
    status_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&VioNode::printStatus, this));

    RCLCPP_INFO(
      get_logger(), "Waiting for the car to stand still for %.1f s to initialize.",
      fp.init_window_s);
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    declare_parameter<std::string>("image_topic", "/camera/fixed/image_raw/compressed");
    declare_parameter<bool>("image_compressed", true);
    declare_parameter<std::string>("odometry_topic", "/ground_vio/odometry");
    declare_parameter<std::string>("world_frame_id", "world");
    declare_parameter<std::string>("imu_frame_id", "imu_link");

    declare_parameter<bool>("camera.calibrated", false);
    declare_parameter<double>("camera.fx", 400.0);
    declare_parameter<double>("camera.fy", 400.0);
    declare_parameter<double>("camera.cx", 320.0);
    declare_parameter<double>("camera.cy", 240.0);
    declare_parameter<std::vector<double>>("camera.distortion", std::vector<double>{0, 0, 0, 0});

    declare_parameter<std::vector<double>>(
      "extrinsic.imu_R_cam", std::vector<double>{0, 0, 1, -1, 0, 0, 0, -1, 0});
    declare_parameter<std::vector<double>>("extrinsic.imu_p_cam", std::vector<double>{0, 0, 0});
    declare_parameter<bool>("extrinsic.estimate", true);
    declare_parameter<double>("extrinsic.rot_sigma_rad", 0.05);
    declare_parameter<double>("extrinsic.pos_sigma_m", 0.03);
    declare_parameter<double>("camera_time_offset_s", 0.0);

    declare_parameter<double>("imu.gyro_noise", 2e-4);
    declare_parameter<double>("imu.accel_noise", 2e-3);
    declare_parameter<double>("imu.gyro_walk", 1e-5);
    declare_parameter<double>("imu.accel_walk", 1e-4);
    declare_parameter<double>("gravity", 9.81);

    declare_parameter<double>("pixel_sigma_px", 1.0);
    declare_parameter<int>("max_clones", 11);
    declare_parameter<int>("min_track_length", 4);
    declare_parameter<double>("min_parallax_deg", 1.0);
    declare_parameter<double>("min_depth_m", 0.2);
    declare_parameter<double>("max_depth_m", 30.0);
    declare_parameter<double>("chi2_scale", 1.0);

    declare_parameter<double>("init_window_s", 1.0);
    declare_parameter<double>("init_gyro_std", 0.03);
    declare_parameter<double>("init_accel_std", 0.3);

    declare_parameter<double>("planar_z_sigma_m", 0.0);
    declare_parameter<bool>("zupt.enabled", false);

    declare_parameter<int>("tracker.max_features", 150);
    declare_parameter<double>("tracker.min_distance_px", 25.0);
  }

  FilterParams readFilterParams()
  {
    FilterParams fp;
    fp.gravity = Eigen::Vector3d(0.0, 0.0, -get_parameter("gravity").as_double());
    fp.imu.gyro_noise = get_parameter("imu.gyro_noise").as_double();
    fp.imu.accel_noise = get_parameter("imu.accel_noise").as_double();
    fp.imu.gyro_walk = get_parameter("imu.gyro_walk").as_double();
    fp.imu.accel_walk = get_parameter("imu.accel_walk").as_double();

    const auto R = get_parameter("extrinsic.imu_R_cam").as_double_array();
    const auto p = get_parameter("extrinsic.imu_p_cam").as_double_array();
    if (R.size() != 9 || p.size() != 3) {
      throw std::runtime_error("extrinsic.imu_R_cam needs 9 values and imu_p_cam needs 3");
    }
    Eigen::Matrix3d Rm;
    Rm << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    fp.q_imu_cam = Eigen::Quaterniond(Rm).normalized();
    fp.p_imu_cam = Eigen::Vector3d(p[0], p[1], p[2]);
    fp.estimate_extrinsics = get_parameter("extrinsic.estimate").as_bool();
    fp.extrinsic_rot_sigma = get_parameter("extrinsic.rot_sigma_rad").as_double();
    fp.extrinsic_pos_sigma = get_parameter("extrinsic.pos_sigma_m").as_double();

    const double focal = 0.5 * (get_parameter("camera.fx").as_double() +
      get_parameter("camera.fy").as_double());
    fp.pixel_sigma = get_parameter("pixel_sigma_px").as_double() / focal;
    fp.max_clones = static_cast<int>(get_parameter("max_clones").as_int());
    fp.min_track_length = static_cast<int>(get_parameter("min_track_length").as_int());
    fp.min_parallax_deg = get_parameter("min_parallax_deg").as_double();
    fp.min_depth = get_parameter("min_depth_m").as_double();
    fp.max_depth = get_parameter("max_depth_m").as_double();
    fp.chi2_scale = get_parameter("chi2_scale").as_double();
    fp.init_window_s = get_parameter("init_window_s").as_double();
    fp.init_gyro_std = get_parameter("init_gyro_std").as_double();
    fp.init_accel_std = get_parameter("init_accel_std").as_double();
    fp.planar_z_sigma = get_parameter("planar_z_sigma_m").as_double();
    fp.zupt_enabled = get_parameter("zupt.enabled").as_bool();
    return fp;
  }

  // Seconds since the first IMU message. The Pi stamps are around 1.8e9 s, which leaves
  // little resolution in a double, so the filter works with small numbers.
  double toFilterTime(const builtin_interfaces::msg::Time & stamp) const
  {
    return static_cast<double>(stamp.sec - t0_sec_) + 1e-9 * static_cast<double>(stamp.nanosec);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!have_t0_) {
      t0_sec_ = msg->header.stamp.sec;
      have_t0_ = true;
    }
    ImuSample s;
    s.t = toFilterTime(msg->header.stamp);
    s.gyro = Eigen::Vector3d(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.accel = Eigen::Vector3d(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    filter_->addImu(s);
    runFilter();
  }

  void onCompressed(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    cv::Mat gray = cv::imdecode(msg->data, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Could not decode an image");
      return;
    }
    onImage(msg->header.stamp, gray);
  }

  void onRaw(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat gray;
    if (msg->encoding == "mono8") {
      gray = cv::Mat(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
        const_cast<uint8_t *>(msg->data.data()), msg->step).clone();
    } else if (msg->encoding == "bgr8" || msg->encoding == "rgb8") {
      cv::Mat color(
        static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
        const_cast<uint8_t *>(msg->data.data()), msg->step);
      cv::cvtColor(color, gray, msg->encoding == "bgr8" ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unsupported image encoding %s", msg->encoding.c_str());
      return;
    }
    onImage(msg->header.stamp, gray);
  }

  void onImage(const builtin_interfaces::msg::Time & stamp, const cv::Mat & gray)
  {
    if (!have_t0_) {
      return;   // no IMU yet, so there is no time axis
    }
    const std::vector<FeatureObservation> obs = tracker_->track(gray);
    filter_->addImage(toFilterTime(stamp) + camera_time_offset_, obs);
    runFilter();
  }

  void runFilter()
  {
    const bool was_initialized = filter_->initialized();
    const int frames_before = filter_->stats().frames;
    filter_->process();
    if (!was_initialized && filter_->initialized()) {
      RCLCPP_INFO(
        get_logger(), "Initialized. Gyroscope bias [%.5f %.5f %.5f] rad/s",
        filter_->gyroBias().x(), filter_->gyroBias().y(), filter_->gyroBias().z());
    }
    if (filter_->stats().frames != frames_before) {
      publishOdometry();
    }
  }

  void publishOdometry()
  {
    nav_msgs::msg::Odometry odom;
    const double t = filter_->time();
    const int64_t sec = static_cast<int64_t>(std::floor(t));
    odom.header.stamp.sec = static_cast<int32_t>(sec + t0_sec_);
    odom.header.stamp.nanosec = static_cast<uint32_t>((t - static_cast<double>(sec)) * 1e9);
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    const Eigen::Vector3d & p = filter_->position();
    const Eigen::Quaterniond & q = filter_->orientation();
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // The twist is given in the child frame, as the message definition asks.
    const Eigen::Vector3d v_body = q.conjugate() * filter_->velocity();
    odom.twist.twist.linear.x = v_body.x();
    odom.twist.twist.linear.y = v_body.y();
    odom.twist.twist.linear.z = v_body.z();

    // Position block from the filter. The rotation block is the covariance of the error in the
    // body frame, which is close to the world frame for small roll and pitch.
    const Eigen::MatrixXd P = filter_->coreCovariance();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        odom.pose.covariance[6 * i + j] = P(3 + i, 3 + j);
        odom.pose.covariance[6 * (3 + i) + (3 + j)] = P(i, j);
      }
    }
    odom_pub_->publish(odom);
  }

  void printStatus()
  {
    if (!filter_->initialized()) {
      RCLCPP_INFO(get_logger(), "Not initialized yet: the car has to stand still.");
      return;
    }
    const FilterStats & s = filter_->stats();
    const Eigen::Vector3d & p = filter_->position();
    RCLCPP_INFO(
      get_logger(),
      "frames %d, position [%.2f %.2f %.2f] m, features used %d, rejected by geometry %d "
      "and by gate %d, extrinsic rotation changed by %.2f deg",
      s.frames, p.x(), p.y(), p.z(), s.features_used, s.rejected_geometry, s.rejected_gate,
      extrinsicChangeDeg());
  }

  double extrinsicChangeDeg()
  {
    if (!have_initial_ext_) {
      initial_ext_ = filter_->extrinsicRotation();
      have_initial_ext_ = true;
    }
    const Eigen::Quaterniond d = initial_ext_.conjugate() * filter_->extrinsicRotation();
    return 2.0 * std::atan2(d.vec().norm(), std::abs(d.w())) * 180.0 / M_PI;
  }

  std::unique_ptr<VioFilter> filter_;
  std::unique_ptr<FeatureTracker> tracker_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  double camera_time_offset_ = 0.0;
  std::string frame_id_;
  std::string child_frame_id_;
  bool have_t0_ = false;
  int32_t t0_sec_ = 0;
  bool have_initial_ext_ = false;
  Eigen::Quaterniond initial_ext_ = Eigen::Quaterniond::Identity();
};

}  // namespace ground_vio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ground_vio::VioNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ground_vio"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
