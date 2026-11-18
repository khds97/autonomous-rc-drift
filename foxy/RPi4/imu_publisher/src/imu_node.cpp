#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher()
  : Node("ism330dhcx_publisher"),
    i2c_fd_(-1),
    gpio_chip_fd_(-1),
    gpio_event_fd_(-1),
    running_(false)
  {
    openI2c();
    try {
      initializeSensor();
      openGpioInterrupt();
    } catch (...) {
      if (gpio_event_fd_ >= 0) {
        close(gpio_event_fd_);
        gpio_event_fd_ = -1;
      }
      if (gpio_chip_fd_ >= 0) {
        close(gpio_chip_fd_);
        gpio_chip_fd_ = -1;
      }
      close(i2c_fd_);
      i2c_fd_ = -1;
      throw;
    }

    raw_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS());

    running_.store(true);
    interrupt_thread_ = std::thread(&ImuPublisher::interruptLoop, this);

    RCLCPP_INFO(
      get_logger(),
      "ISM330DHCX interrupt publisher started: gpiochip0 line 23 (INT1), "
      "publishing uncalibrated data on /imu/data_raw.");
  }

  ~ImuPublisher() override
  {
    running_.store(false);
    if (interrupt_thread_.joinable()) {
      interrupt_thread_.join();
    }

    if (i2c_fd_ >= 0) {
      try {
        writeRegister(INT1_CTRL, 0x00);
        writeRegister(CTRL1_XL, 0x00);
        writeRegister(CTRL2_G, 0x00);
      } catch (...) {
      }
      close(i2c_fd_);
    }
    if (gpio_event_fd_ >= 0) {
      close(gpio_event_fd_);
    }
    if (gpio_chip_fd_ >= 0) {
      close(gpio_chip_fd_);
    }
  }

private:
  static constexpr uint8_t I2C_ADDRESS = 0x6A;
  static constexpr uint8_t WHO_AM_I = 0x0F;
  static constexpr uint8_t INT1_CTRL = 0x0D;
  static constexpr uint8_t CTRL1_XL = 0x10;
  static constexpr uint8_t CTRL2_G = 0x11;
  static constexpr uint8_t CTRL3_C = 0x12;
  static constexpr uint8_t OUTX_L_G = 0x22;
  static constexpr unsigned int GPIO_CHIP_LINE = 23;

  static constexpr double ACCEL_SENSITIVITY = 0.000061;
  static constexpr double GYRO_SENSITIVITY = 0.00875;
  static constexpr double GRAVITY = 9.80665;
  static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

  int i2c_fd_;
  int gpio_chip_fd_;
  int gpio_event_fd_;
  std::atomic<bool> running_;
  std::thread interrupt_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_publisher_;

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
    writeRegister(INT1_CTRL, 0x03); // Accelerometer and gyroscope data ready on INT1
  }

  void openGpioInterrupt()
  {
    gpio_chip_fd_ = open("/dev/gpiochip0", O_RDONLY);
    if (gpio_chip_fd_ < 0) {
      throw std::runtime_error("Could not open /dev/gpiochip0");
    }

    struct gpioevent_request request = {};
    request.lineoffset = GPIO_CHIP_LINE;
    request.handleflags = GPIOHANDLE_REQUEST_INPUT;
    request.eventflags = GPIOEVENT_REQUEST_RISING_EDGE;
    std::strncpy(
      request.consumer_label, "ism330dhcx_int1",
      sizeof(request.consumer_label) - 1);

    if (ioctl(gpio_chip_fd_, GPIO_GET_LINEEVENT_IOCTL, &request) < 0) {
      throw std::runtime_error("Could not request rising-edge event on GPIO23");
    }
    gpio_event_fd_ = request.fd;
  }

  static int16_t toSigned(uint8_t low, uint8_t high)
  {
    const uint16_t value =
      (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    return static_cast<int16_t>(value);
  }

  void publishImu(uint64_t event_timestamp_ns)
  {
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

    const double gyro_x = gx_raw * GYRO_SENSITIVITY * DEG_TO_RAD;
    const double gyro_y = gy_raw * GYRO_SENSITIVITY * DEG_TO_RAD;
    const double gyro_z = gz_raw * GYRO_SENSITIVITY * DEG_TO_RAD;

    // GPIO event timestamps use CLOCK_MONOTONIC. Map that timestamp to the
    // node clock so it can be compared with ROS messages from the same host.
    const auto ros_now = get_clock()->now();
    const int64_t steady_now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const int64_t event_age_ns =
      steady_now_ns - static_cast<int64_t>(event_timestamp_ns);
    const rclcpp::Time event_stamp(
      ros_now.nanoseconds() - event_age_ns, ros_now.get_clock_type());

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = event_stamp;
    msg.header.frame_id = "imu_link";
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity.x = gyro_x;
    msg.angular_velocity.y = gyro_y;
    msg.angular_velocity.z = gyro_z;
    msg.linear_acceleration.x = ax_raw * ACCEL_SENSITIVITY * GRAVITY;
    msg.linear_acceleration.y = ay_raw * ACCEL_SENSITIVITY * GRAVITY;
    msg.linear_acceleration.z = az_raw * ACCEL_SENSITIVITY * GRAVITY;

    raw_publisher_->publish(msg);
  }

  void interruptLoop()
  {
    struct pollfd pfd = {};
    pfd.fd = gpio_event_fd_;
    pfd.events = POLLIN;

    while (running_.load()) {
      const int poll_result = poll(&pfd, 1, 100);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "GPIO event poll failed");
        break;
      }
      if (poll_result == 0) {
        continue;
      }

      struct gpioevent_data event = {};
      const ssize_t bytes_read = read(gpio_event_fd_, &event, sizeof(event));
      if (bytes_read != static_cast<ssize_t>(sizeof(event))) {
        if (running_.load()) {
          RCLCPP_WARN(get_logger(), "Could not read GPIO event");
        }
        continue;
      }

      try {
        publishImu(event.timestamp);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "IMU read failed: %s", e.what());
      }
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
