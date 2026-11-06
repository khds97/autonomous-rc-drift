cmake_minimum_required(VERSION 3.5)
project(driftmodel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driftmodel_msg REQUIRED)
find_package(Threads REQUIRED)

# pigpio daemon interface (apt: pigpio / libpigpiod-if-dev, run `sudo pigpiod` before starting the node)
find_library(PIGPIOD_IF2 pigpiod_if2)

if(PIGPIOD_IF2)
  message(STATUS "pigpiod_if2 found. Building driftmodel_node.")

  add_executable(driftmodel_node
    src/driftmodel_node.cpp
    src/driver.cpp
    src/ADS1115.cpp
    src/I2Cdev.cpp
    src/pubSensorData.cpp
    src/pwm.cpp
    src/wheelSpeed.cpp)
  target_include_directories(driftmodel_node PRIVATE include)
  ament_target_dependencies(driftmodel_node rclcpp driftmodel_msg)
  target_link_libraries(driftmodel_node ${PIGPIOD_IF2} Threads::Threads)

  install(TARGETS driftmodel_node
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "pigpiod_if2 not found. Skipping driftmodel_node.")
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
