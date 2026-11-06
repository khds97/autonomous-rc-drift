#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "driver.hpp"
#include "pubSensorData.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the driver.
  auto driver_ = std::make_shared<driver>();

  // Create the node and run it.
  auto node = std::make_shared<pubSensorData>(driver_);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
