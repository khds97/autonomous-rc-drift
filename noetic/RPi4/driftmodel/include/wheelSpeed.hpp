#ifndef WHEELSPEED_HPP_
#define WHEELSPEED_HPP_

#include <iostream>
#include <pigpiod_if2.h>

class wheelSpeed
{  
public:
  wheelSpeed();
  ~wheelSpeed();
  
  void encoderPulse(int pi, unsigned gpio, unsigned level, std::uint32_t tick);
  void encoderPulseLeft(int pi, unsigned gpio, unsigned level, std::uint32_t tick);
  void encoderPulseRight(int pi, unsigned gpio, unsigned level, std::uint32_t tick);
  static void _pulseEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user);
  static void _pulseLeftEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user);
  static void _pulseRightEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user);
  float getLeft();
  float getRight();
  float getPos();
  void setZero();
  void calcRpm();
  
private:
  static float encoderPos;
  static float encoderLeftPos;
  static float encoderRightPos;
  static float rpm;
  static float rl_speed;
  static float rr_speed;
};

#endif
