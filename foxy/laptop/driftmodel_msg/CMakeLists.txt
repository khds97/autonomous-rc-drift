cmake_minimum_required(VERSION 3.5)
project(driftmodel_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sensors.msg"
  "msg/CtrlVal.msg"
  "msg/PoseFlag.msg"
  "srv/Flag.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
