cmake_minimum_required(VERSION 3.0.2)
project(quadruped_control)

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
## Messages (msg) and services (srv) of this package
## ------------------------------------------------------------------
##add_message_files(
 ## FILES
  ##HighLevelCmd.msg
##)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

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
  scripts/policy_node_onnx.py
  scripts/policy_node_old.py
  # Uncomment if you have them in this folder:
  # scripts/gait_manager.py
  # scripts/ik_node.py
  # scripts/vel_cmd_listener.py
  # scripts/auto_walk_4legs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

