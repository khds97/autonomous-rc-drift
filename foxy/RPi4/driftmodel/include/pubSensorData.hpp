#ifndef PUBSENSORDATA_HPP_
#define PUBSENSORDATA_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "driftmodel_msg/msg/sensors.hpp"
#include "driver.hpp"

// ROS 2 node that publishes the vehicle sensor data on `sensors`.
//
// Parameters:
//   publish_rate_hz       publish rate (default 34.0)
//   speed_window_s        averaging window of the speed estimate (default 0.1)
//   motor_counts_per_rev  encoder steps per motor revolution (default 6.0)
//   magnets_per_wheel     magnets on one wheel = pulses per wheel revolution (default 8.0)
//   rpm_timeout_s         no pulse for this long -> wheel speed 0 (default 0.5)
class pubSensorData : public rclcpp::Node
{
public:
  explicit pubSensorData(std::shared_ptr<driver> driver);
  ~pubSensorData();

  void getAllData();

  void deinitialize_driver();

private:
  std::shared_ptr<driver> driver_;

  rclcpp::Publisher<driftmodel_msg::msg::Sensors>::SharedPtr sensors_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // PUBSENSORDATA_HPP_
