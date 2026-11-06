#include <pigpiod_if2.h>

#include <cstdio>
#include <iostream>
#include <memory>

#include "driver.hpp"

driver::driver()
: m_pigpio_handle(-1), last_current_(0.0f)
{
}

driver::~driver()
{
  deinit();
}

void driver::configure(const wheelSpeed::Config & cfg)
{
  wheel_cfg_ = cfg;
}

int driver::init()
{
  if (m_pigpio_handle >= 0) {
    return 0;
  }

  const int pi = pigpio_start(NULL, NULL);
  if (pi < 0) {
    std::cerr << "could not connect to pigpiod (is `sudo pigpiod` running?)" << std::endl;
    return 1;
  }
  m_pigpio_handle = pi;

  pwm_ = std::make_shared<pwm>();
  wheelSpeed_ = std::make_shared<wheelSpeed>(wheel_cfg_);
  adc0_ = std::make_shared<ADS1115>();

  const unsigned inputs[] = {
    pins::ENCODER_A, pins::ENCODER_B, pins::ENCODER_C,
    pins::HALL_FL, pins::HALL_FR, pins::HALL_RL, pins::HALL_RR,
    pins::TIRE};
  for (unsigned gpio : inputs) {
    set_mode(m_pigpio_handle, gpio, PI_INPUT);
  }

  // pull up is needed as the encoder common is grounded; the hall sensors read high until a magnet is close
  const unsigned pulled_up[] = {
    pins::ENCODER_A, pins::ENCODER_B, pins::ENCODER_C,
    pins::HALL_FL, pins::HALL_FR, pins::HALL_RL, pins::HALL_RR};
  for (unsigned gpio : pulled_up) {
    set_pull_up_down(m_pigpio_handle, gpio, PI_PUD_UP);
  }

  // start the motor decoder from the real pin levels so that the first edge is not misread
  wheelSpeed_->setInitialLevels(
    gpio_read(m_pigpio_handle, pins::ENCODER_A), gpio_read(m_pigpio_handle, pins::ENCODER_B),
    gpio_read(m_pigpio_handle, pins::ENCODER_C));

  // The user pointer must be the object whose member function the callback calls.
  struct Binding
  {
    unsigned gpio;
    unsigned edge;
    CBFuncEx_t func;
    void * user;
  };
  const Binding bindings[] = {
    {pins::ENCODER_A, EITHER_EDGE, wheelSpeed::_pulseEx, wheelSpeed_.get()},
    {pins::ENCODER_B, EITHER_EDGE, wheelSpeed::_pulseEx, wheelSpeed_.get()},
    {pins::ENCODER_C, EITHER_EDGE, wheelSpeed::_pulseEx, wheelSpeed_.get()},
    // falling edge only: one magnet pass = one pulse
    {pins::HALL_FL, FALLING_EDGE, wheelSpeed::_hallEx, wheelSpeed_.get()},
    {pins::HALL_FR, FALLING_EDGE, wheelSpeed::_hallEx, wheelSpeed_.get()},
    {pins::HALL_RL, FALLING_EDGE, wheelSpeed::_hallEx, wheelSpeed_.get()},
    {pins::HALL_RR, FALLING_EDGE, wheelSpeed::_hallEx, wheelSpeed_.get()},
    {pins::TIRE, EITHER_EDGE, pwm::_pulseExTire, pwm_.get()}};
  for (const Binding & b : bindings) {
    const int id = callback_ex(m_pigpio_handle, b.gpio, b.edge, b.func, b.user);
    if (id < 0) {
      std::cerr << "could not register the callback of GPIO " << b.gpio << std::endl;
      deinit();
      return 1;
    }
    callback_ids_.push_back(id);
  }

  return 0;
}

driver::data driver::sample()
{
  data d;

  // the pulse times are pigpio ticks, so "now" has to come from the same clock
  wheelSpeed_->calcRpm(get_current_tick(m_pigpio_handle));
  d.rpm = wheelSpeed_->getPos();
  d.fl_speed = wheelSpeed_->getWheel(wheelSpeed::FL);
  d.fr_speed = wheelSpeed_->getWheel(wheelSpeed::FR);
  d.rl_speed = wheelSpeed_->getWheel(wheelSpeed::RL);
  d.rr_speed = wheelSpeed_->getWheel(wheelSpeed::RR);

  float volts = 0.0f;
  if (adc0_->readVoltage(0, &volts)) {
    last_current_ = volts;
  }
  d.current = last_current_;   // keep the last good value when a read fails

  d.tire_pwm = pwm_->getTire();

  return d;
}

int driver::deinit()
{
  if (m_pigpio_handle < 0) {
    return 0;
  }
  for (int id : callback_ids_) {
    callback_cancel(id);
  }
  callback_ids_.clear();

  std::cout << "shutting down RPi" << std::endl;
  pigpio_stop(m_pigpio_handle);
  m_pigpio_handle = -1;
  return 0;
}
