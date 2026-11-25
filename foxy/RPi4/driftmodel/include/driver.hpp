#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include <atomic>
#include <memory>
#include <thread>
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

  // The ADS1115 read blocks for about 10 ms, so it runs in its own thread and sample() only
  // returns the latest value. This keeps the publish timer and its timestamp free of the delay.
  void adcLoop();
  std::thread adc_thread_;
  std::atomic<bool> adc_running_;
  std::atomic<float> last_current_;
};

#endif  // DRIVER_HPP_
