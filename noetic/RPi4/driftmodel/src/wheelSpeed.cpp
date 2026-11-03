#include <iostream>
#include <pigpiod_if2.h>
#include <chrono>

#define HALL_L1 19 
#define HALL_R1 6
#define HALL_L2 13
#define HALL_R2 5

#define ENCODER_A 7
#define ENCODER_B 25
#define ENCODER_C 8

#include "wheelSpeed.hpp"

float wheelSpeed::encoderPos;
float wheelSpeed::encoderLeftPos;
float wheelSpeed::encoderRightPos;

float wheelSpeed::rpm;
float wheelSpeed::rl_speed;
float wheelSpeed::rr_speed;

wheelSpeed::wheelSpeed(){
  
}

wheelSpeed::~wheelSpeed(){

}
void wheelSpeed::setZero(){
	encoderPos = 0;
	encoderLeftPos = 0;
	encoderRightPos = 0;
}

void wheelSpeed::calcRpm(){

	
	static std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point EndTime = std::chrono::system_clock::now();
	std::chrono::milliseconds mill = std::chrono::duration_cast<std::chrono::milliseconds>(EndTime - StartTime);
	
	if (mill.count() >= 100){

		rpm = std::abs(((encoderPos / 3)/0.1) * 60);
		rl_speed = ((encoderLeftPos / 4)/0.1) * 60;
		rr_speed = ((encoderRightPos / 4)/0.1) * 60;

		StartTime = std::chrono::system_clock::now();
	}
}

float wheelSpeed::getPos(){

	return rpm;

}

float wheelSpeed::getRight(){

	return rr_speed;
}

float wheelSpeed::getLeft(){

	return rl_speed;
}

void wheelSpeed::encoderPulseLeft(int pi, unsigned gpio, unsigned level, std::uint32_t tick)
{
   static int levA=0, levB=0;

   if (gpio == HALL_L1){
     levA = level;
     //std::cout<<"HALL_L1: "<<levA<<std::endl;
     }
   else{
     levB = level;
     //std::cout<<"HALL_L2: "<<levB<<std::endl;
     }

   //std::cout<<"levLA: "<<levA<<" levLB: "<<levB<<'\n';
   
      if ((gpio == HALL_L1) && (levA == 1)){
        if (levB == 0){
	 encoderLeftPos++;
	 //std::cout<<"pos_lRF="<<pos_L<<std::endl;
	}
      }
      else if ((gpio == HALL_L1) && (levA == 0)){

       if (levB == 0){
	 encoderLeftPos++;
	 //std::cout<<"pos_lFR"<<pos_L<<std::endl;
	}
       }
      if ((gpio == HALL_L2) && (levB == 1)){

        if (levA == 0){
	 encoderLeftPos++;
	 //std::cout<<"pos_lRR="<<pos_L<<std::endl;
	}
       }
      else if ((gpio == HALL_L2) && (levB == 0)){
  
        if (levA == 0){
	 encoderLeftPos++;
	 //std::cout<<"pos_lFF="<<pos_L<<std::endl;
	}
      }
}

void wheelSpeed::encoderPulseRight(int pi, unsigned gpio, unsigned level, std::uint32_t tick)
{
   static int levA=0, levB=0;

   if (gpio == HALL_R1){
     levA = level;
     //std::cout<<"HALL_R1: "<<levA<<std::endl;
     }
   else{
     levB = level;
     //std::cout<<"HALL_R2: "<<levB<<std::endl;
     }
   //std::cout<<"levRA: "<<levA<<" levRB: "<<levB<<'\n';
      if ((gpio == HALL_R1) && (levA == 1)){
        if (levB == 0){
	 encoderRightPos++;
	 //std::cout<<"pos_rRF="<<pos_R<<std::endl;
	}
      }
      else if ((gpio == HALL_R1) && (levA == 0)){

       if (levB == 0){
	 encoderRightPos++;
	 //std::cout<<"pos_rFR"<<pos_R<<std::endl;
	}
       }
      if ((gpio == HALL_R2) && (levB == 1)){

        if (levA == 0){
	 encoderRightPos++;
	 //std::cout<<"pos_rRR="<<pos_R<<std::endl;
	}
       }
      else if ((gpio == HALL_R2) && (levB == 0)){
  
        if (levA == 0){
	 encoderRightPos++;
	 //std::cout<<"pos_rFF="<<pos_R<<std::endl;
	}
      }   
}

void wheelSpeed::encoderPulse(int pi, unsigned gpio, unsigned level, std::uint32_t tick)
{
   /*
   
--------+             +-------------+             +-------------+          0
        |             |             |             |             |
   A    |             |             |             |             |
        |             |             |             |             |
        +-------------+             +-------------+             +--------- 1

---+             +-------------+             +-------------+          0
   |             |             |             |             |
   |             |             |             |             |
   |             |             |             |             |
   +-------------+             +-------------+             +--------- 1

       +-------------+             +-------------+                0
       |             |             |             |
   C   |             |             |             |
       |             |             |             |
-------+             +-------------+             +-------------+  1
   
  */
   
   static int levA=0, levB=0, levC=0, lastGpio = -1;

   if (gpio == ENCODER_A) levA = level; else if (gpio == ENCODER_B) levB = level; else levC = level;

   if (gpio != lastGpio) /* debounce */
   {
      lastGpio = gpio;

      if ((gpio == ENCODER_A) && (level == 0))
      {
         if (!levB) ++encoderPos;
      }
      
      else if ((gpio == ENCODER_B) && (level == 0))
      {
         if (!levC) ++encoderPos;
      }
      
      else if ((gpio == ENCODER_C) && (level == 0))
      {
         if (!levA) ++encoderPos;
      }
      
      
      else if ((gpio == ENCODER_B) && (level == 1))
      {
         if (levA) --encoderPos;
      }
      
      else if ((gpio == ENCODER_C) && (level == 1))
      {
         if (levB) --encoderPos;
      }
      
      else if ((gpio == ENCODER_A) && (level == 1))
      {
         if (levC) --encoderPos;
      }
   }
}

void wheelSpeed::_pulseLeftEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   wheelSpeed *mySelf = (wheelSpeed *) user;

   mySelf->encoderPulseLeft(pi, gpio, level, tick); /* Call the instance callback. */
}

void wheelSpeed::_pulseRightEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   wheelSpeed *mySelf = (wheelSpeed *) user;

   mySelf->encoderPulseRight(pi, gpio, level, tick); /* Call the instance callback. */
}

void wheelSpeed::_pulseEx(int pi, unsigned gpio, unsigned level, std::uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   wheelSpeed *mySelf = (wheelSpeed *) user;

   mySelf->encoderPulse(pi, gpio, level, tick); /* Call the instance callback. */
}
