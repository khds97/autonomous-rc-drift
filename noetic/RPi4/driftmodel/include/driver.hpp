#ifndef DRIVER_HPP_
#define DRIVER_HPP_

#include <memory>
#include <functional>

#include "pwm.hpp"
#include "wheelSpeed.hpp"
#include "ADS1115.h"

class driver
{
public:
  driver();
  ~driver();
  
  int init();
  
  struct data
  {
    float current, rpm, rl_speed, rr_speed, tire_pwm;
  };
    
  void set_data_callback(std::function<void(driver::data)> callback);
  
  void getAll();
  //static float dt;
  int deinit();
  
private:
  int m_pigpio_handle;
  
  std::shared_ptr<pwm> pwm_;
  
  std::shared_ptr<wheelSpeed> wheelSpeed_;
  
  std::shared_ptr<ADS1115> adc0_;
  
  std::function<void(driver::data)> m_data_callback;
  
  float zero;
};

#endif
