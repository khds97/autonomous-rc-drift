#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include <memory>
#include <vector>

#include "ADS1115.h"
#include "pwm.hpp"
#include "wheelSpeed.hpp"

// Owns the GPIO / ADC resources and returns one set of sensor readings per call to sample().
class driver
{
public:
  driver();
  ~driver();

  struct data
  {
    float current, rpm, fl_speed, fr_speed, rl_speed, rr_speed, tire_pwm;
  };

  // Set the wheel speed parameters. Must be called before init().
  void configure(const wheelSpeed::Config & cfg);

  // Connects to pigpiod and starts the GPIO callbacks. Returns 0 on success.
  int init();

  // Reads all sensors. Call at the publish rate.
  data sample();

  // Stops the callbacks and disconnects from pigpiod. Safe to call more than once.
  int deinit();

private:
  int m_pigpio_handle;
  wheelSpeed::Config wheel_cfg_;

  std::shared_ptr<pwm> pwm_;
  std::shared_ptr<wheelSpeed> wheelSpeed_;
  std::shared_ptr<ADS1115> adc0_;

  std::vector<int> callback_ids_;
  float last_current_;
};

#endif  // DRIVER_HPP_
