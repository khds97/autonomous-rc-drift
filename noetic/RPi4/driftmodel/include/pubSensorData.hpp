#ifndef PUBSENSORDATA_HPP_
#define PUBSENSORDATA_HPP_

#include <memory>
#include <functional>

#include "driver.hpp"
#include <ros/ros.h>

class pubSensorData
{
public:

  pubSensorData(std::shared_ptr<driver> driver, int argc, char **argv);
  
  ~pubSensorData();
  
  void getAllData(driver::data sensorData);
  
  void spin();
  
  void deinitialize_driver();

private:

  std::shared_ptr<driver> driver_;
  
  std::shared_ptr<ros::NodeHandle> node_;
  
  ros::Publisher sensors_Pub;

};

#endif
