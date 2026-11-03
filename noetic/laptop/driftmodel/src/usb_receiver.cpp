#include <boost/python.hpp>
#include "usb_receiver.hpp"
#include <serial/serial.h>
#include <iostream>
#include <cstdio>
#include <chrono>
#include <thread>
#include "ros/ros.h"

serial::Serial my_serial;
    
float USBReceiver::steer;
float USBReceiver::throttle;
bool USBReceiver::mode_flag;
bool USBReceiver::punishment_flag;

USBReceiver::USBReceiver()
{
   my_serial.setPort("/dev/ACMSTM32");
   my_serial.setBaudrate(115200);
   serial::Timeout to = serial::Timeout::simpleTimeout(1000);
   my_serial.setTimeout(to);
   my_serial.open();
}

USBReceiver::~USBReceiver()
{
    my_serial.close();
}

void USBReceiver::receiveData()
{
    /*steer = 0.0;
    throttle = 0.0;*/
    std::string received_data = my_serial.read(my_serial.available()); 
    
    sscanf(const_cast<char*>(received_data.c_str()), "%f\t%d\t%f\t%d\n", &steer, &mode_flag, &throttle, &punishment_flag);

}

void USBReceiver::transmitData(float steer_, float throttle_)
{
    
    if(steer_ > 3.3) steer_ = 3.3;
   
    else if(throttle_ > 3.3) throttle_ = 3.3;
    
    else if(steer_ < 0) steer_ = 0;
    
    else if(throttle_ < 0) throttle = 0;
    
    ROS_INFO("Steer: %f, Throttle: %f", steer_, throttle_);
    
    std::string dataToSend = std::to_string(steer_) + "\n" + std::to_string(throttle_) + "\n";

    my_serial.write(dataToSend);

}

float USBReceiver::steerVal()
{
    return steer;
}

float USBReceiver::throttleVal()
{
    return throttle;
}

bool USBReceiver::flagVal_mode()
{
    return mode_flag;
}

bool USBReceiver::flagVal_punishment()
{
    return punishment_flag;
}

BOOST_PYTHON_MODULE(usb_receiver)
{
  boost::python::class_<USBReceiver>("USBReceiver", boost::python::init<>()).def("receiveData", &USBReceiver::receiveData).def("transmitData", &USBReceiver::transmitData).def("steer", &USBReceiver::steerVal).def("throttle", &USBReceiver::throttleVal).def("mode_flag", &USBReceiver::flagVal_mode).def("punishment_flag", &USBReceiver::flagVal_punishment).staticmethod("steer").staticmethod("throttle").staticmethod("mode_flag").staticmethod("punishment_flag");
}
