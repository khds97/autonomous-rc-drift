cmake_minimum_required(VERSION 3.5)
project(camera_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(dual_camera_publisher
  src/dual_camera_publisher.cpp)

target_include_directories(dual_camera_publisher PRIVATE
  ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(dual_camera_publisher
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge)

target_link_libraries(dual_camera_publisher
  ${OpenCV_LIBRARIES}
  Threads::Threads)

install(TARGETS dual_camera_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch media
  DESTINATION share/${PROJECT_NAME})

ament_package()
