#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <cstdio>

#include "pubSensorData.hpp"

#include "driver.hpp"

#include <functional>
#include <memory>

#define HALL_L1 19 
#define HALL_R1 6
#define HALL_L2 13
#define HALL_R2 5

#define ENCODER_A 7
#define ENCODER_B 25
#define ENCODER_C 8

#define TIRE 21

driver::driver()
{
  driver::m_pigpio_handle = -1;

}

driver::~driver(){
}

int driver::init(){  
   
   int pi = pigpio_start(NULL, NULL);
   if (pi < 0) return 1;
   
   driver::m_pigpio_handle = pi;
   
   driver::pwm_ = std::make_shared<pwm>();
   
   driver::wheelSpeed_ = std::make_shared<wheelSpeed>();
   
   driver::adc0_ = std::make_shared<ADS1115>();
   
   set_mode(driver::m_pigpio_handle, ENCODER_A, PI_INPUT);
   set_mode(driver::m_pigpio_handle, ENCODER_B, PI_INPUT);
   set_mode(driver::m_pigpio_handle, ENCODER_C, PI_INPUT);
   
   set_mode(driver::m_pigpio_handle, HALL_L1, PI_INPUT);
   set_mode(driver::m_pigpio_handle, HALL_R1, PI_INPUT);
   set_mode(driver::m_pigpio_handle, HALL_L2, PI_INPUT);
   set_mode(driver::m_pigpio_handle, HALL_R2, PI_INPUT);
   
   set_mode(driver::m_pigpio_handle, TIRE, PI_INPUT);
   /* pull up is needed as encoder common is grounded */

   set_pull_up_down(driver::m_pigpio_handle, ENCODER_A, PI_PUD_UP);
   set_pull_up_down(driver::m_pigpio_handle, ENCODER_B, PI_PUD_UP);
   set_pull_up_down(driver::m_pigpio_handle, ENCODER_C, PI_PUD_UP);

   set_pull_up_down(driver::m_pigpio_handle, HALL_L1, PI_PUD_UP);
   set_pull_up_down(driver::m_pigpio_handle, HALL_R1, PI_PUD_UP);
   set_pull_up_down(driver::m_pigpio_handle, HALL_L2, PI_PUD_UP);
   set_pull_up_down(driver::m_pigpio_handle, HALL_R2, PI_PUD_UP);
      
   /* monitor encoder level changes */
   
   callback_ex(driver::m_pigpio_handle, ENCODER_A, EITHER_EDGE, driver::wheelSpeed_->_pulseEx, this);
   callback_ex(driver::m_pigpio_handle, ENCODER_B, EITHER_EDGE, driver::wheelSpeed_->_pulseEx, this);
   callback_ex(driver::m_pigpio_handle, ENCODER_C, EITHER_EDGE, driver::wheelSpeed_->_pulseEx, this);
   
   callback_ex(driver::m_pigpio_handle, HALL_L1, EITHER_EDGE, driver::wheelSpeed_->_pulseLeftEx, this);
   callback_ex(driver::m_pigpio_handle, HALL_R1, EITHER_EDGE, driver::wheelSpeed_->_pulseRightEx, this);
   callback_ex(driver::m_pigpio_handle, HALL_L2, EITHER_EDGE, driver::wheelSpeed_->_pulseLeftEx, this);
   callback_ex(driver::m_pigpio_handle, HALL_R2, EITHER_EDGE, driver::wheelSpeed_->_pulseRightEx, this);
      
   callback_ex(driver::m_pigpio_handle, TIRE, EITHER_EDGE, driver::pwm_->_pulseExTire, this);
   
}

void driver::set_data_callback(std::function<void (data)> callback)
{
	//std::cout << "getting all datas..." << "\n";
	
	driver::data data;
	
	driver::wheelSpeed_->calcRpm();
	
	//std::cout<<"Left: "<<driver::wheelSpeed_->getLeft()<<'\n';
	
	//velocity = ((rpm * 0.09283) * 2 * 3.14 * 0.026);
	data.rpm = std::abs(driver::wheelSpeed_->getPos());
	data.rl_speed = driver::wheelSpeed_->getLeft();
	data.rr_speed = driver::wheelSpeed_->getRight();

	data.current = driver::adc0_->getVal(0)*0.0001875;
	
	data.tire_pwm = driver::pwm_->getTire();
	
	callback(data);
	
	driver::wheelSpeed_->setZero();

}

int driver::deinit(){

   std::cout<<"shutting down RPi"<<std::endl;
   pigpio_stop(driver::m_pigpio_handle);
 
}
