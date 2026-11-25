cmake_minimum_required(VERSION 3.8)
project(ground_vio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Build only the filter, the tracker and the tests, without ROS:
#   cmake -S . -B build -DGROUND_VIO_STANDALONE=ON && cmake --build build && ctest --test-dir build
option(GROUND_VIO_STANDALONE "Build without ROS" OFF)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video calib3d)

add_library(ground_vio_core STATIC src/vio_filter.cpp)
target_include_directories(ground_vio_core PUBLIC include)
target_link_libraries(ground_vio_core PUBLIC Eigen3::Eigen)

add_library(ground_vio_tracker STATIC src/feature_tracker.cpp)
target_include_directories(ground_vio_tracker PUBLIC include)
target_link_libraries(ground_vio_tracker PUBLIC ground_vio_core ${OpenCV_LIBS})
target_include_directories(ground_vio_tracker PUBLIC ${OpenCV_INCLUDE_DIRS})

if(GROUND_VIO_STANDALONE OR BUILD_TESTING)
  enable_testing()
  foreach(name test_jacobians test_filter_sim test_tracker)
    add_executable(${name} test/${name}.cpp)
    target_include_directories(${name} PRIVATE test)
    target_link_libraries(${name} ground_vio_tracker)
    add_test(NAME ${name} COMMAND ${name})
  endforeach()
endif()

if(NOT GROUND_VIO_STANDALONE)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)

  add_executable(vio_node src/vio_node.cpp)
  target_link_libraries(vio_node ground_vio_tracker)
  ament_target_dependencies(vio_node rclcpp sensor_msgs nav_msgs)

  install(TARGETS vio_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS scripts/calibrate_intrinsics.py DESTINATION lib/${PROJECT_NAME})
  ament_package()
endif()
