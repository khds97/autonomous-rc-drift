#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "pubSensorData.hpp"

pubSensorData::pubSensorData(std::shared_ptr<driver> driver)
: rclcpp::Node("driftmodel_node"), driver_(std::move(driver))
{
  this->declare_parameter("publish_rate_hz", 34.0);
  this->declare_parameter("speed_window_s", 0.1);
  this->declare_parameter("motor_counts_per_rev", 6.0);
  this->declare_parameter("magnets_per_wheel", 8.0);
  this->declare_parameter("rpm_timeout_s", 0.5);

  const double rate_hz = this->get_parameter("publish_rate_hz").as_double();
  if (rate_hz <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be positive");
  }

  wheelSpeed::Config cfg;
  cfg.window_s = this->get_parameter("speed_window_s").as_double();
  cfg.motor_counts_per_rev = this->get_parameter("motor_counts_per_rev").as_double();
  cfg.magnets_per_wheel = this->get_parameter("magnets_per_wheel").as_double();
  cfg.rpm_timeout_s = this->get_parameter("rpm_timeout_s").as_double();
  if (cfg.magnets_per_wheel <= 0.0) {
    throw std::invalid_argument("magnets_per_wheel must be positive");
  }

  sensors_pub_ = this->create_publisher<driftmodel_msg::msg::Sensors>("sensors", 10);

  driver_->configure(cfg);
  if (driver_->init() != 0) {
    RCLCPP_FATAL(this->get_logger(), "driver initialization failed");
    throw std::runtime_error("driver initialization failed");
  }

  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz)),
    std::bind(&pubSensorData::getAllData, this));

  RCLCPP_INFO(this->get_logger(), "successfully initialized");
}

pubSensorData::~pubSensorData()
{
  deinitialize_driver();
}

void pubSensorData::getAllData()
{
  const driver::data sensorData = driver_->sample();

  driftmodel_msg::msg::Sensors message;
  message.header.stamp = this->now();
  message.header.frame_id = "base_link";
  message.rpm = sensorData.rpm;
  message.tire_pwm = static_cast<int32_t>(sensorData.tire_pwm);
  message.fl_speed = sensorData.fl_speed;
  message.fr_speed = sensorData.fr_speed;
  message.rl_speed = sensorData.rl_speed;
  message.rr_speed = sensorData.rr_speed;
  message.current = sensorData.current;

  sensors_pub_->publish(message);
}

void pubSensorData::deinitialize_driver()
{
  driver_->deinit();
  RCLCPP_INFO(this->get_logger(), "driver successfully deinitialized");
}
