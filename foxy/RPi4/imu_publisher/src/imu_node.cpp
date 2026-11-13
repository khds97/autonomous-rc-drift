#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <functional>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher() : Node("ism330dhcx_publisher"), i2c_fd_(-1)
  {
    openI2c();
    try {
      initializeSensor();
    } catch (...) {
      close(i2c_fd_);
      i2c_fd_ = -1;
      throw;
    }

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(
      10ms, std::bind(&ImuPublisher::publishImu, this));

    RCLCPP_INFO(get_logger(), "Publishing on /imu/data_raw");
  }

  ~ImuPublisher() override
  {
    if (i2c_fd_ >= 0) {
      try {
        writeRegister(CTRL1_XL, 0x00);
        writeRegister(CTRL2_G, 0x00);
      } catch (...) {
      }
      close(i2c_fd_);
    }
  }

private:
  static constexpr uint8_t I2C_ADDRESS = 0x6A;
  static constexpr uint8_t WHO_AM_I = 0x0F;
  static constexpr uint8_t CTRL1_XL = 0x10;
  static constexpr uint8_t CTRL2_G = 0x11;
  static constexpr uint8_t CTRL3_C = 0x12;
  static constexpr uint8_t STATUS_REG = 0x1E;
  static constexpr uint8_t OUTX_L_G = 0x22;

  static constexpr double ACCEL_SENSITIVITY = 0.000061;
  static constexpr double GYRO_SENSITIVITY = 0.00875;
  static constexpr double GRAVITY = 9.80665;
  static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

  int i2c_fd_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void openI2c()
  {
    i2c_fd_ = open("/dev/i2c-0", O_RDWR);
    if (i2c_fd_ < 0) {
      throw std::runtime_error("Could not open /dev/i2c-0");
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, I2C_ADDRESS) < 0) {
      close(i2c_fd_);
      i2c_fd_ = -1;
      throw std::runtime_error("Could not select I2C address 0x6A");
    }
  }

  void writeRegister(uint8_t reg, uint8_t value)
  {
    const uint8_t data[2] = {reg, value};
    if (write(i2c_fd_, data, sizeof(data)) != static_cast<ssize_t>(sizeof(data))) {
      throw std::runtime_error("I2C register write failed");
    }
  }

  bool readRegisters(uint8_t reg, uint8_t * data, uint8_t length)
  {
    struct i2c_msg messages[2];
    messages[0].addr = I2C_ADDRESS;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;
    messages[1].addr = I2C_ADDRESS;
    messages[1].flags = I2C_M_RD;
    messages[1].len = length;
    messages[1].buf = data;

    struct i2c_rdwr_ioctl_data transaction;
    transaction.msgs = messages;
    transaction.nmsgs = 2;
    return ioctl(i2c_fd_, I2C_RDWR, &transaction) >= 0;
  }

  uint8_t readRegister(uint8_t reg)
  {
    uint8_t value = 0;
    if (!readRegisters(reg, &value, 1)) {
      throw std::runtime_error("I2C register read failed");
    }
    return value;
  }

  void initializeSensor()
  {
    if (readRegister(WHO_AM_I) != 0x6B) {
      throw std::runtime_error("Unexpected WHO_AM_I value");
    }
    writeRegister(CTRL1_XL, 0x40);  // 104 Hz, +/-2 g
    writeRegister(CTRL2_G, 0x40);   // 104 Hz, +/-250 dps
    writeRegister(CTRL3_C, 0x44);   // BDU and IF_INC
  }

  static int16_t toSigned(uint8_t low, uint8_t high)
  {
    const uint16_t value =
      (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    return static_cast<int16_t>(value);
  }

  void publishImu()
  {
    try {
      const uint8_t status = readRegister(STATUS_REG);
      if ((status & 0x03) != 0x03) {
        return;
      }

      uint8_t data[12] = {};
      if (!readRegisters(OUTX_L_G, data, sizeof(data))) {
        throw std::runtime_error("Could not read IMU output registers");
      }

      const int16_t gx_raw = toSigned(data[0], data[1]);
      const int16_t gy_raw = toSigned(data[2], data[3]);
      const int16_t gz_raw = toSigned(data[4], data[5]);
      const int16_t ax_raw = toSigned(data[6], data[7]);
      const int16_t ay_raw = toSigned(data[8], data[9]);
      const int16_t az_raw = toSigned(data[10], data[11]);

      sensor_msgs::msg::Imu msg;
      msg.header.stamp = get_clock()->now();
      msg.header.frame_id = "imu_link";
      msg.orientation_covariance[0] = -1.0;

      msg.angular_velocity.x = gx_raw * GYRO_SENSITIVITY * DEG_TO_RAD;
      msg.angular_velocity.y = gy_raw * GYRO_SENSITIVITY * DEG_TO_RAD;
      msg.angular_velocity.z = gz_raw * GYRO_SENSITIVITY * DEG_TO_RAD;

      msg.linear_acceleration.x = ax_raw * ACCEL_SENSITIVITY * GRAVITY;
      msg.linear_acceleration.y = ay_raw * ACCEL_SENSITIVITY * GRAVITY;
      msg.linear_acceleration.z = az_raw * ACCEL_SENSITIVITY * GRAVITY;

      publisher_->publish(msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "IMU read failed: %s", e.what());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuPublisher>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("imu_publisher"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
