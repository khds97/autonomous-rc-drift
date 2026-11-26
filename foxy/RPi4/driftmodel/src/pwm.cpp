#include "pwm.hpp"

std::atomic<float> pwm::pwm_tire{0.0f};

pwm::pwm() {}

pwm::~pwm() {}

float pwm::getTire()
{
  return pwm_tire;
}

void pwm::getTirePWM(int, unsigned, unsigned level, std::uint32_t tick)
{
  static std::uint32_t start_tick_ = 0;

  if (level == 1) {
    start_tick_ = tick;
  } else if (level == 0) {
    // unsigned subtraction handles the wrap-around of the 32-bit tick counter
    pwm_tire = static_cast<float>(tick - start_tick_);
  }
}

void pwm::_pulseExTire(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void * user)
{
  pwm * mySelf = static_cast<pwm *>(user);
  mySelf->getTirePWM(pi, gpio, level, tick);
}
