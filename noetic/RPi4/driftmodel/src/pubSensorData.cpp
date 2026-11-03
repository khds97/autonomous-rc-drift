#include <iostream>
#include <memory>
#include <utility>
#include <ros/ros.h>

#include <functional>

#include "pubSensorData.hpp"
#include "driver.hpp"

#include <driftmodel_msg/sensors.h>

pubSensorData::pubSensorData(std::shared_ptr<driver> driver, int argc, char **argv)
{
  pubSensorData::driver_ = driver;
  
  ros::init(argc, argv, "driftmodel_node");
  
  pubSensorData::node_ = std::make_shared<ros::NodeHandle>();
  
  pubSensorData::sensors_Pub = pubSensorData::node_->advertise<driftmodel_msg::sensors>("sensors", 10);
  
  pubSensorData::driver_->init();
  
  ROS_INFO_STREAM("successfully initialized");

}

pubSensorData::~pubSensorData()
{
}

void pubSensorData::getAllData(driver::data sensorData){
  
  //pubSensorData::driver_->getAll();
  //std::cout << "Trying to publish..." << "\n";
  
  ros::Time time = ros::Time::now();
    
  driftmodel_msg::sensors message;
  message.rpm = sensorData.rpm;
  message.tire_pwm = sensorData.tire_pwm;
  message.rl_speed = sensorData.rl_speed;
  message.rr_speed = sensorData.rr_speed;
  message.current = sensorData.current;
  message.header.stamp = time;
  message.header.frame_id = "base_link";
  
  pubSensorData::sensors_Pub.publish(message);
  

}

void pubSensorData::spin()
{
    ros::Rate loop_rate(34);
    while (ros::ok())
    {
        pubSensorData::driver_->set_data_callback(std::bind(&pubSensorData::getAllData, this, std::placeholders::_1));

        ros::spinOnce();

        loop_rate.sleep();
    }
    // Deinitialize driver.
    pubSensorData::deinitialize_driver();
}

void pubSensorData::deinitialize_driver()
{
    pubSensorData::driver_->deinit();
    ROS_INFO_STREAM("driver successfully deinitialized");
}

