#include <iostream>
#include <pigpiod_if2.h>

#include "pwm.hpp"

float pwm::pwm_tire;

pwm::pwm(){ 
 
}

pwm::~pwm(){

}

float pwm::getTire(){
	return pwm_tire;
}

void pwm::getTirePWM(int pi, unsigned gpio, unsigned level, uint32_t tick){

	static uint32_t start_tick_ = 0;
	
	if (level == 1){
		start_tick_ = tick;
	}
	
	else if (level == 0){
		pwm_tire = tick - start_tick_;
	}
	
	//pwm_tire_d = pwm_tire / (0.1);

}

void pwm::_pulseExTire(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user)
{
   //pwm* mySelf = static_cast<pwm*>(user);
   
   pwm *mySelf = (pwm *) user;
   
   mySelf->getTirePWM(pi, gpio, level, tick); /* Call the instance callback. */
}
