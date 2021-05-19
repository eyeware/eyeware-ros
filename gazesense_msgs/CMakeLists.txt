#
# Copyright (c) 2021, Eyeware Tech SA
#

cmake_minimum_required(VERSION 3.0.2)
project(gazesense_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BlinkInfo.msg
  Ray3D.msg
  GazeInfo.msg
  HeadPoseInfo.msg
  ScreenGazeInfo.msg
  TrackedPersonInfo.msg
  TrackedUserArray.msg
  TrackedUser.msg
  TrackingConfidence.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## catkin specific configuration
catkin_package(
 CATKIN_DEPENDS rospy std_msgs geometry_msgs
)

## Build
##
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)