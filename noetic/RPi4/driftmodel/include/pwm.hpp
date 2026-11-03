#ifndef PWM_HPP_
#define PWM_HPP_

#include <iostream>
#include <pigpiod_if2.h>

class pwm
{
public:
  pwm();
  ~pwm();
  void getTirePWM(int pi, unsigned gpio, unsigned level, uint32_t tick);
  static void _pulseExTire(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user);
  float getTire();
private:
  static float pwm_tire;

};
#endif
