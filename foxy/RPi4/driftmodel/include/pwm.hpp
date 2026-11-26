#ifndef PWM_HPP_
#define PWM_HPP_

#include <atomic>
#include <cstdint>

// Measures the pulse width of a PWM signal from the rising / falling edge ticks.
class pwm
{
public:
  pwm();
  ~pwm();

  // Edge handler (called from the pigpio callback thread). tick is in microseconds.
  void getTirePWM(int pi, unsigned gpio, unsigned level, std::uint32_t tick);

  // Static callback that is registered with pigpio. `user` must be the pwm object.
  static void _pulseExTire(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void * user);

  // Last measured pulse width [us].
  float getTire();

private:
  static std::atomic<float> pwm_tire;
};

#endif  // PWM_HPP_
