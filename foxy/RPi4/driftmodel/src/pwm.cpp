#include "pwm.hpp"

pwm::pwm() {}

pwm::~pwm() {}

float pwm::getTire() const
{
  return static_cast<float>(width_us_.load(std::memory_order_relaxed));
}

void pwm::getTirePWM(unsigned, unsigned level, std::uint32_t tick)
{
  if (level == 1) {
    start_tick_ = tick;
  } else if (level == 0) {
    // unsigned subtraction handles the wrap-around of the 32-bit tick counter
    width_us_.store(tick - start_tick_, std::memory_order_relaxed);
  }
}

void pwm::_pulseExTire(int, unsigned gpio, unsigned level, std::uint32_t tick, void * user)
{
  static_cast<pwm *>(user)->getTirePWM(gpio, level, tick);
}
