cmake_minimum_required(VERSION 3.0.2)
project(rpi_stepper_motor_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  message_generation
)

catkin_python_setup()

add_action_files(
  DIRECTORY
    action
  FILES
    ControlMotor.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/rpi_stepper_motor_driver/motor_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  action
  launch
  config
  src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)