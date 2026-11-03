cmake_minimum_required(VERSION 3.0.2)
project(driftmodel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  driftmodel_msg
)

find_package(Boost REQUIRED COMPONENTS python REQUIRED)
find_package(PythonLibs 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usb_receiver
  CATKIN_DEPENDS
    roscpp
    serial
    driftmodel_msg
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  src/pose_flag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_library(usb_receiver src/usb_receiver.cpp)

set_target_properties(usb_receiver PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_link_libraries(usb_receiver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
