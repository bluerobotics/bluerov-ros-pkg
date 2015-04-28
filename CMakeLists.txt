cmake_minimum_required(VERSION 2.8.3)
project(bluerov)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

## Declare ROS messages and services
add_message_files(FILES Thruster.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pilot src/pilot.cpp)
target_link_libraries(pilot ${catkin_LIBRARIES})
add_dependencies(pilot bluerov_generate_messages_cpp)
