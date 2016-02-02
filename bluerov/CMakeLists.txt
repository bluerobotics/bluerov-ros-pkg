cmake_minimum_required(VERSION 2.8.3)
project(bluerov)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  joy
  mavros
)

## Declare a catkin package
catkin_package(
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
