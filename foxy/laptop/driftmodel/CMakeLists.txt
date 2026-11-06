cmake_minimum_required(VERSION 3.5)
project(driftmodel)

find_package(ament_cmake REQUIRED)

# Executable scripts (run with `ros2 run driftmodel <name>.py`)
install(PROGRAMS
  src/dagger.py
  src/train_agent.py
  src/pose_flag.py
  src/sync_sub.py
  src/usb_receiver/record.py
  DESTINATION lib/${PROJECT_NAME})

# Modules imported by the scripts. They are installed next to the scripts.
install(FILES
  src/model.py
  src/preprocess.py
  src/utils.py
  src/usb_receiver.py
  src/driftmodel_env.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES demo.rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
