cmake_minimum_required(VERSION 3.0.2)
project(driftmodel_msg)

# Find catkin macros and libraries
find_package(catkin REQUIRED 
  roscpp
  std_msgs
  message_generation
)

# Add message files.
add_message_files(
  FILES
  sensors.msg
)

# Generate messages.
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Set up catkin package.
catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)
