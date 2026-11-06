#ifndef WHEELSPEED_HPP_
#define WHEELSPEED_HPP_

// Wheel / motor speed measurement.
//
// This header does not depend on pigpio or ROS, so the decoding and speed
// estimation can be compiled and tested on any machine.

#include <atomic>
#include <cstdint>
#include <deque>
#include <utility>

// GPIO numbers (BCM) used by the sensors.
namespace pins
{
constexpr unsigned HALL_FL = 19;    // front-left wheel hall sensor
constexpr unsigned HALL_FR = 5;     // front-right wheel hall sensor
constexpr unsigned HALL_RL = 13;    // rear-left wheel hall sensor
constexpr unsigned HALL_RR = 6;     // rear-right wheel hall sensor
constexpr unsigned ENCODER_A = 7;   // 3-phase motor encoder
constexpr unsigned ENCODER_B = 25;
constexpr unsigned ENCODER_C = 8;
constexpr unsigned TIRE = 21;       // PWM signal measured by pwm
}  // namespace pins

// Rate [counts/s] of a cumulative counter over a sliding time window.
// The rate is computed from the real elapsed time, so it does not depend on
// how often update() is called.
class RateEstimator
{
public:
  explicit RateEstimator(double window_s = 0.1) : window_s_(window_s) {}

  // total_count: cumulative (signed) count, t_s: monotonic time in seconds.
  double update(int64_t total_count, double t_s);
  void reset() { history_.clear(); }

private:
  double window_s_;
  std::deque<std::pair<double, int64_t>> history_;
};

// Speed of one wheel from the time between two magnet passes (one hall sensor per wheel).
// pulse() is called from the GPIO callback thread with the pigpio tick [us] of a magnet pass
// (falling edge), rpm() from another thread. Last pulse time and period are stored in one
// atomic word, so a reader never sees a mix of two pulses.
class PulsePeriod
{
public:
  void pulse(std::uint32_t tick_us);

  // now_us: current pigpio tick [us]. Returns 0 when no pulse arrived within timeout_us.
  // The result is capped by the time since the last pulse, so the value falls smoothly while a
  // wheel slows down instead of holding the last period until the timeout.
  double rpm(std::uint32_t now_us, double magnets_per_rev, std::uint32_t timeout_us) const;

private:
  // Upper 32 bits: tick of the last pulse. Lower 32 bits: time between the last two pulses [us].
  // A period of 0 means that no period has been measured yet.
  std::atomic<std::uint64_t> packed_{0};
  bool has_last_ = false;                  // used by the callback thread only
};

// 3-phase (A, B, C) hall decoder. Counts one step per valid state change of the
// 6-step sequence 1 -> 3 -> 2 -> 6 -> 4 -> 5 (state = A | B << 1 | C << 2).
class HallDecoder
{
public:
  explicit HallDecoder(bool invert = false) : invert_(invert) {}

  void setLevels(int a, int b, int c) { state_ = (a ? 1 : 0) | (b ? 2 : 0) | (c ? 4 : 0); }
  void update(unsigned channel, int level);   // channel: 0 = A, 1 = B, 2 = C
  int64_t count() const { return count_.load(std::memory_order_relaxed); }

private:
  bool invert_;
  int state_ = 0;
  std::atomic<int64_t> count_{0};
};

class wheelSpeed
{
public:
  enum Wheel { FL = 0, FR = 1, RL = 2, RR = 3, NUM_WHEELS = 4 };

  struct Config
  {
    double window_s = 0.1;                // averaging window of the motor speed estimate
    double motor_counts_per_rev = 6.0;    // 6 steps per electrical revolution (multiply by pole pairs for mechanical rpm)
    double magnets_per_wheel = 8.0;       // magnets on one wheel = pulses per wheel revolution
    double rpm_timeout_s = 0.5;           // no pulse for this long -> wheel speed 0
  };

  wheelSpeed();                                // default configuration
  explicit wheelSpeed(const Config & cfg);
  ~wheelSpeed();

  // Call once after the GPIO pins are configured, before enabling the callbacks.
  void setInitialLevels(int enc_a, int enc_b, int enc_c);

  // Edge handlers (called from the pigpio callback thread).
  void encoderPulse(unsigned gpio, unsigned level);
  void hallPulse(unsigned gpio, unsigned level, std::uint32_t tick);   // acts on falling edges only

  // Static callbacks that are registered with pigpio. `user` must be the wheelSpeed object.
  static void _pulseEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void * user);
  static void _hallEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void * user);

  // Update the speed estimates. Call once per publish cycle with the current pigpio tick [us]
  // (get_current_tick()), so that the pulse times and "now" use the same clock.
  void calcRpm(std::uint32_t now_tick_us);

  float getPos() const { return rpm_; }                          // motor rpm, unsigned
  float getWheel(Wheel w) const { return wheel_rpm_[w]; }        // wheel rpm, unsigned (one sensor per wheel: no direction)

private:
  Config cfg_;
  HallDecoder motor_;
  RateEstimator motor_rate_;
  PulsePeriod wheels_[NUM_WHEELS];
  float rpm_ = 0.0f;
  float wheel_rpm_[NUM_WHEELS] = {0.0f, 0.0f, 0.0f, 0.0f};

  bool have_tick_ = false;
  std::uint32_t last_tick_ = 0;
  double elapsed_s_ = 0.0;                                       // unwrapped time for the motor estimate
};

#endif  // WHEELSPEED_HPP_
