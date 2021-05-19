#
# Copyright (c) 2021, Eyeware Tech SA
#

cmake_minimum_required(VERSION 3.0.2)
project(gazesense_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  gazesense_msgs
)

## catkin specific configuration
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs gazesense_msgs
)

## Build
##
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)