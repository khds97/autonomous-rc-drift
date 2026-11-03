#include "driver.hpp"
#include "pubSensorData.hpp"

int main(int argc, char **argv)
{
    // Create the driver.
    auto driver_ = std::make_shared<driver>();

    // Create the node.
    pubSensorData node(driver_, argc, argv);

    // Run the node.
    node.spin();
}
