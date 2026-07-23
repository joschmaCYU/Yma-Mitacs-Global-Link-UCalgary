cmake_minimum_required(VERSION 3.0.2)
project(f_quadruped_control)

## ------------------------------------------------------------------
## Catkin dependencies and messages
## ------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()

## ------------------------------------------------------------------
## Package export
## ------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
)

## ------------------------------------------------------------------
## Includes
## ------------------------------------------------------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## ------------------------------------------------------------------
## Python scripts installation
##  NOTE: Keep ONLY the ones that EXIST in scripts/
## ------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/policy_node.py
  scripts/policy_to_controllers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

