// imu_node.cpp
// Adafruit ISM330DHCX (I2C) -> sensor_msgs/Imu on /imu/data_raw
// ROS2 Foxy, C++, interrupt driven (data-ready on INT1)
//
// Wiring: sensor INT1 -> a Raspberry Pi GPIO (default BCM 23, header pin 16).
// The sensor raises a short pulse on INT1 for every new gyro sample. A dedicated
// thread blocks on the GPIO edge (libgpiod), reads the sample over I2C and
// publishes it. The message stamp is the kernel's timestamp of the edge, so it
// is the real sample time and does not depend on any timer or poll period.
//
// Sensor setup:
//   ODR        : 208 Hz (accel and gyro)
//   accel range: +-4 g      (0.122 mg/LSB)
//   gyro range : +-1000 dps (35 mdps/LSB)
//
// Build: needs libgpiod (sudo apt install libgpiod-dev gpiod), link with -lgpiod.
// Test the wiring first with:  gpiomon --rising-edge gpiochip0 23
//
// Parameters:
//   i2c_device       (string, default "/dev/i2c-0")
//   frame_id         (string, default "imu_link")
//   gpio_chip        (string, default "gpiochip0")
//   gpio_line        (int,    default 23)    line offset on that chip (BCM number on a Pi 4)
//   event_timeout_ms (int,    default 200)   warn if no edge arrives for this long

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>

#include <fcntl.h>
#include <gpiod.h>
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
  ImuPublisher()
  : Node("ism330dhcx_publisher"),
    i2c_fd_(-1)
  {
    declare_parameter("i2c_device", std::string("/dev/i2c-0"));
    declare_parameter("frame_id", std::string("imu_link"));
    declare_parameter("gpio_chip", std::string("gpiochip0"));
    declare_parameter("gpio_line", 23);
    declare_parameter("event_timeout_ms", 200);

    i2c_device_ = get_parameter("i2c_device").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    const std::string gpio_chip = get_parameter("gpio_chip").as_string();
    const int gpio_line = get_parameter("gpio_line").as_int();
    event_timeout_ms_ = get_parameter("event_timeout_ms").as_int();
    if (event_timeout_ms_ < 10) {
      event_timeout_ms_ = 10;
    }

    openI2c();

    try {
      setupGpio(gpio_chip, gpio_line);
      initializeSensor();
    } catch (...) {
      cleanup();
      throw;
    }

    raw_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS());

    last_rate_log_ = std::chrono::steady_clock::now();

    running_ = true;
    thread_ = std::thread(&ImuPublisher::irqLoop, this);

    RCLCPP_INFO(
      get_logger(),
      "ISM330DHCX started: ODR 208 Hz, +-4 g, +-1000 dps, DRDY on %s line %d, "
      "publishing uncalibrated data on /imu/data_raw",
      gpio_chip.c_str(), gpio_line);
  }

  ~ImuPublisher() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }

    if (i2c_fd_ >= 0) {
      try {
        writeRegister(INT1_CTRL, 0x00);
        writeRegister(CTRL1_XL, 0x00);
        writeRegister(CTRL2_G, 0x00);
      } catch (...) {
      }
    }
    cleanup();
  }

private:
  static constexpr uint8_t I2C_ADDRESS = 0x6A;

  static constexpr uint8_t COUNTER_BDR_REG1 = 0x0B;
  static constexpr uint8_t INT1_CTRL = 0x0D;
  static constexpr uint8_t WHO_AM_I = 0x0F;
  static constexpr uint8_t CTRL1_XL = 0x10;
  static constexpr uint8_t CTRL2_G = 0x11;
  static constexpr uint8_t CTRL3_C = 0x12;
  static constexpr uint8_t OUTX_L_G = 0x22;   // gyro XYZ, then accel XYZ

  // COUNTER_BDR_REG1 bit 7: data-ready as a short pulse (not latched), so every
  // new sample produces its own rising edge
  static constexpr uint8_t DATAREADY_PULSED = 0x80;
  // INT1_CTRL bit 1: gyro data-ready on INT1. Accel runs at the same ODR, so one
  // edge per sample pair is enough
  static constexpr uint8_t INT1_DRDY_G = 0x02;

  // CTRL1_XL / CTRL2_G: ODR in bits 7:4 (0101 = 208 Hz), full scale in bits 3:2
  //   accel: 00 = 2 g, 01 = 16 g, 10 = 4 g, 11 = 8 g
  //   gyro : 00 = 250, 01 = 500, 10 = 1000, 11 = 2000 dps
  static constexpr uint8_t CTRL1_XL_VALUE = 0x58;   // 208 Hz, +-4 g
  static constexpr uint8_t CTRL2_G_VALUE = 0x58;    // 208 Hz, +-1000 dps
  // CTRL3_C: BDU (bit 6) + IF_INC (bit 2)
  static constexpr uint8_t CTRL3_C_VALUE = 0x44;
  static constexpr uint8_t CTRL3_C_SW_RESET = 0x01;

  static constexpr double ACCEL_SENSITIVITY = 0.000122;   // g per LSB at +-4 g
  static constexpr double GYRO_SENSITIVITY = 0.035;       // deg/s per LSB at 1000 dps
  static constexpr double GRAVITY = 9.80665;

  static constexpr double DEG_TO_RAD =
    3.14159265358979323846 / 180.0;

  int i2c_fd_;
  std::string i2c_device_;
  std::string frame_id_;
  int event_timeout_ms_ = 200;

  gpiod_chip * chip_ = nullptr;
  gpiod_line * line_ = nullptr;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_publisher_;

  std::thread thread_;
  std::atomic<bool> running_{false};

  // Published-rate report (only touched by the IRQ thread)
  std::chrono::steady_clock::time_point last_rate_log_;
  uint32_t published_count_ = 0;
  uint32_t missed_count_ = 0;

  void cleanup()
  {
    if (line_ != nullptr) {
      gpiod_line_release(line_);
      line_ = nullptr;
    }
    if (chip_ != nullptr) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
      i2c_fd_ = -1;
    }
  }

  void openI2c()
  {
    i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);

    if (i2c_fd_ < 0) {
      throw std::runtime_error("Could not open " + i2c_device_);
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, I2C_ADDRESS) < 0) {
      close(i2c_fd_);
      i2c_fd_ = -1;

      throw std::runtime_error("Could not select I2C address 0x6A");
    }

    RCLCPP_INFO(
      get_logger(), "I2C device opened: %s (fd=%d)", i2c_device_.c_str(), i2c_fd_);
  }

  void setupGpio(const std::string & chip_name, int line_offset)
  {
    chip_ = gpiod_chip_open_by_name(chip_name.c_str());
    if (chip_ == nullptr) {
      throw std::runtime_error("Could not open GPIO chip " + chip_name);
    }

    line_ = gpiod_chip_get_line(chip_, static_cast<unsigned int>(line_offset));
    if (line_ == nullptr) {
      throw std::runtime_error("Could not get GPIO line " + std::to_string(line_offset));
    }

    if (gpiod_line_request_rising_edge_events(line_, "ism330dhcx_imu") < 0) {
      throw std::runtime_error(
        "Could not request rising edge events on GPIO line " +
        std::to_string(line_offset) + " (line busy or no permission?)");
    }
  }

  void writeRegister(uint8_t reg, uint8_t value)
  {
    const uint8_t data[2] = {reg, value};

    if (write(i2c_fd_, data, sizeof(data)) !=
      static_cast<ssize_t>(sizeof(data)))
    {
      throw std::runtime_error("I2C register write failed");
    }
  }

  bool readRegisters(uint8_t reg, uint8_t * data, uint8_t length)
  {
    struct i2c_msg messages[2] = {};

    messages[0].addr = I2C_ADDRESS;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    messages[1].addr = I2C_ADDRESS;
    messages[1].flags = I2C_M_RD;
    messages[1].len = length;
    messages[1].buf = data;

    struct i2c_rdwr_ioctl_data transaction = {};

    transaction.msgs = messages;
    transaction.nmsgs = 2;

    if (ioctl(i2c_fd_, I2C_RDWR, &transaction) < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "I2C_RDWR failed: reg=0x%02X, errno=%d, %s",
        reg, errno, strerror(errno));

      return false;
    }

    return true;
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

    // Software reset so the configuration starts from a known state
    writeRegister(CTRL3_C, CTRL3_C_SW_RESET);
    bool reset_done = false;
    for (int i = 0; i < 50; ++i) {
      std::this_thread::sleep_for(2ms);
      if ((readRegister(CTRL3_C) & CTRL3_C_SW_RESET) == 0) {
        reset_done = true;
        break;
      }
    }
    if (!reset_done) {
      throw std::runtime_error("Sensor software reset did not finish");
    }

    writeRegister(CTRL3_C, CTRL3_C_VALUE);

    // Interrupt routing first, then start the sampling
    writeRegister(COUNTER_BDR_REG1, DATAREADY_PULSED);
    writeRegister(INT1_CTRL, INT1_DRDY_G);

    writeRegister(CTRL1_XL, CTRL1_XL_VALUE);
    writeRegister(CTRL2_G, CTRL2_G_VALUE);

    RCLCPP_INFO(get_logger(), "ISM330DHCX initialization complete");
  }

  static int16_t toSigned(uint8_t low, uint8_t high)
  {
    const uint16_t value =
      (static_cast<uint16_t>(high) << 8) |
      static_cast<uint16_t>(low);

    return static_cast<int16_t>(value);
  }

  // Edge timestamp from the kernel, converted to the node clock. Depending on the
  // kernel the edge time is CLOCK_REALTIME or CLOCK_MONOTONIC, so test both and
  // use the one that gives a plausible age. If neither does, use the current time.
  rclcpp::Time stampFromEvent(const gpiod_line_event & ev)
  {
    const rclcpp::Time now = get_clock()->now();
    const int64_t event_ns =
      static_cast<int64_t>(ev.ts.tv_sec) * 1000000000LL + ev.ts.tv_nsec;

    constexpr int64_t kMinAgeNs = -5000000;    // edge may not be in the future
    constexpr int64_t kMaxAgeNs = 50000000;    // and not older than 50 ms

    const int64_t real_age = clockNs(CLOCK_REALTIME) - event_ns;
    const int64_t mono_age = clockNs(CLOCK_MONOTONIC) - event_ns;

    int64_t age_ns;
    if (real_age >= kMinAgeNs && real_age <= kMaxAgeNs) {
      age_ns = real_age;
    } else if (mono_age >= kMinAgeNs && mono_age <= kMaxAgeNs) {
      age_ns = mono_age;
    } else {
      return now;
    }
    return rclcpp::Time(now.nanoseconds() - age_ns, now.get_clock_type());
  }

  static int64_t clockNs(clockid_t id)
  {
    timespec t;
    clock_gettime(id, &t);
    return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
  }

  void irqLoop()
  {
    timespec timeout;
    timeout.tv_sec = event_timeout_ms_ / 1000;
    timeout.tv_nsec = static_cast<long>(event_timeout_ms_ % 1000) * 1000000L;
    const timespec no_wait = {0, 0};

    while (running_) {
      const int r = gpiod_line_event_wait(line_, &timeout);
      if (r < 0) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "GPIO event wait failed: %s", strerror(errno));
        std::this_thread::sleep_for(100ms);
        continue;
      }
      if (r == 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No data-ready edge for %d ms: check the INT1 wiring and gpio_line",
          event_timeout_ms_);
        continue;
      }

      gpiod_line_event ev;
      if (gpiod_line_event_read(line_, &ev) != 0) {
        continue;
      }

      // If we fell behind, edges are queued. The data registers only hold the
      // newest sample, so use the newest edge and count the rest as missed.
      while (gpiod_line_event_wait(line_, &no_wait) == 1) {
        if (gpiod_line_event_read(line_, &ev) != 0) {
          break;
        }
        ++missed_count_;
      }

      try {
        publishSample(stampFromEvent(ev));
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "IMU read failed: %s", e.what());
      }
    }
  }

  void publishSample(const rclcpp::Time & stamp)
  {
    uint8_t data[12];
    if (!readRegisters(OUTX_L_G, data, sizeof(data))) {
      return;
    }

    const int16_t gx_raw = toSigned(data[0], data[1]);
    const int16_t gy_raw = toSigned(data[2], data[3]);
    const int16_t gz_raw = toSigned(data[4], data[5]);

    const int16_t ax_raw = toSigned(data[6], data[7]);
    const int16_t ay_raw = toSigned(data[8], data[9]);
    const int16_t az_raw = toSigned(data[10], data[11]);

    sensor_msgs::msg::Imu msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    // This message carries no orientation estimate
    msg.orientation_covariance[0] = -1.0;

    msg.angular_velocity.x = gx_raw * GYRO_SENSITIVITY * DEG_TO_RAD;
    msg.angular_velocity.y = gy_raw * GYRO_SENSITIVITY * DEG_TO_RAD;
    msg.angular_velocity.z = gz_raw * GYRO_SENSITIVITY * DEG_TO_RAD;

    msg.linear_acceleration.x = ax_raw * ACCEL_SENSITIVITY * GRAVITY;
    msg.linear_acceleration.y = ay_raw * ACCEL_SENSITIVITY * GRAVITY;
    msg.linear_acceleration.z = az_raw * ACCEL_SENSITIVITY * GRAVITY;

    raw_publisher_->publish(msg);

    // Report the achieved rate every 5 s (should be close to 208 Hz)
    ++published_count_;
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_rate_log_).count();
    if (elapsed >= 5.0) {
      RCLCPP_INFO(
        get_logger(), "IMU publish rate: %.1f Hz, missed samples: %u",
        published_count_ / elapsed, missed_count_);
      published_count_ = 0;
      missed_count_ = 0;
      last_rate_log_ = now;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<ImuPublisher>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ism330dhcx_publisher"), "%s", e.what());
  }

  rclcpp::shutdown();

  return 0;
}
