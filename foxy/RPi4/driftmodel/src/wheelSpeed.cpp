#include "wheelSpeed.hpp"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// RateEstimator
// ---------------------------------------------------------------------------
double RateEstimator::update(int64_t total_count, double t_s)
{
  history_.emplace_back(t_s, total_count);

  // Keep the newest sample that is at least window_s old (or the oldest one available).
  while (history_.size() > 2 && (t_s - history_[1].first) >= window_s_) {
    history_.pop_front();
  }

  const double dt = t_s - history_.front().first;
  if (dt <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(total_count - history_.front().second) / dt;
}

// ---------------------------------------------------------------------------
// PulsePeriod
// ---------------------------------------------------------------------------
void PulsePeriod::pulse(std::uint32_t tick_us)
{
  const std::uint64_t old = packed_.load(std::memory_order_relaxed);
  const std::uint32_t last = static_cast<std::uint32_t>(old >> 32);
  std::uint32_t period = static_cast<std::uint32_t>(old & 0xFFFFFFFFu);

  if (has_last_) {
    const std::uint32_t p = tick_us - last;   // unsigned: the 32-bit tick wrap is fine
    if (p == 0) {
      return;                                  // two edges with the same time stamp: ignore
    }
    period = p;
  }
  has_last_ = true;
  packed_.store((static_cast<std::uint64_t>(tick_us) << 32) | period, std::memory_order_relaxed);
}

double PulsePeriod::rpm(std::uint32_t now_us, double magnets_per_rev, std::uint32_t timeout_us) const
{
  const std::uint64_t v = packed_.load(std::memory_order_relaxed);
  const std::uint32_t last = static_cast<std::uint32_t>(v >> 32);
  const std::uint32_t period = static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
  if (period == 0) {
    return 0.0;
  }

  // A pulse can arrive between reading "now" and reading the pulse time: treat that as "just now".
  std::int32_t since = static_cast<std::int32_t>(now_us - last);
  if (since < 0) {
    since = 0;
  }
  if (static_cast<std::uint32_t>(since) > timeout_us) {
    return 0.0;
  }

  const double effective_us = std::max(static_cast<double>(period), static_cast<double>(since));
  return 60.0 * 1000000.0 / (effective_us * magnets_per_rev);
}

// ---------------------------------------------------------------------------
// HallDecoder
// ---------------------------------------------------------------------------
namespace
{
// Position of each 3-bit hall state in the sequence 1 -> 3 -> 2 -> 6 -> 4 -> 5. -1 = invalid state (000, 111).
const int8_t kHallPosition[8] = {-1, 0, 2, 1, 4, 5, 3, -1};
}  // namespace

void HallDecoder::update(unsigned channel, int level)
{
  if (channel > 2) {
    return;
  }
  const int bit = 1 << channel;
  const int next = level ? (state_ | bit) : (state_ & ~bit);

  const int p_old = kHallPosition[state_];
  const int p_new = kHallPosition[next];
  state_ = next;
  if (p_old < 0 || p_new < 0) {
    return;
  }

  const int diff = (p_new - p_old + 6) % 6;
  int delta = 0;
  if (diff == 1) {
    delta = 1;
  } else if (diff == 5) {
    delta = -1;
  }
  if (delta != 0) {
    count_.fetch_add(invert_ ? -delta : delta, std::memory_order_relaxed);
  }
}

// ---------------------------------------------------------------------------
// wheelSpeed
// ---------------------------------------------------------------------------
wheelSpeed::wheelSpeed()
: wheelSpeed(Config())
{
}

wheelSpeed::wheelSpeed(const Config & cfg)
: cfg_(cfg),
  motor_(false),
  motor_rate_(cfg.window_s)
{
}

wheelSpeed::~wheelSpeed() {}

void wheelSpeed::setInitialLevels(int enc_a, int enc_b, int enc_c)
{
  motor_.setLevels(enc_a, enc_b, enc_c);
}

void wheelSpeed::encoderPulse(unsigned gpio, unsigned level)
{
  if (gpio == pins::ENCODER_A) {
    motor_.update(0, level);
  } else if (gpio == pins::ENCODER_B) {
    motor_.update(1, level);
  } else if (gpio == pins::ENCODER_C) {
    motor_.update(2, level);
  }
}

void wheelSpeed::hallPulse(unsigned gpio, unsigned level, std::uint32_t tick)
{
  if (level != 0) {
    return;   // the sensor reads low while a magnet is close: one falling edge = one magnet pass
  }
  if (gpio == pins::HALL_FL) {
    wheels_[FL].pulse(tick);
  } else if (gpio == pins::HALL_FR) {
    wheels_[FR].pulse(tick);
  } else if (gpio == pins::HALL_RL) {
    wheels_[RL].pulse(tick);
  } else if (gpio == pins::HALL_RR) {
    wheels_[RR].pulse(tick);
  }
}

void wheelSpeed::_pulseEx(int, unsigned gpio, unsigned level, std::uint32_t, void * user)
{
  static_cast<wheelSpeed *>(user)->encoderPulse(gpio, level);
}

void wheelSpeed::_hallEx(int, unsigned gpio, unsigned level, std::uint32_t tick, void * user)
{
  static_cast<wheelSpeed *>(user)->hallPulse(gpio, level, tick);
}

void wheelSpeed::calcRpm(std::uint32_t now_tick_us)
{
  // Time in seconds since the first call, without the wrap-around of the 32-bit tick.
  if (have_tick_) {
    elapsed_s_ += static_cast<double>(static_cast<std::uint32_t>(now_tick_us - last_tick_)) * 1e-6;
  }
  have_tick_ = true;
  last_tick_ = now_tick_us;

  const double motor_cps = motor_rate_.update(motor_.count(), elapsed_s_);
  rpm_ = static_cast<float>(std::fabs(motor_cps) / cfg_.motor_counts_per_rev * 60.0);

  const std::uint32_t timeout_us = static_cast<std::uint32_t>(cfg_.rpm_timeout_s * 1e6);
  for (int i = 0; i < NUM_WHEELS; ++i) {
    wheel_rpm_[i] = static_cast<float>(wheels_[i].rpm(now_tick_us, cfg_.magnets_per_wheel, timeout_us));
  }
}
