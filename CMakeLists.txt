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
  joy
#  mavros
  message_generation
  dynamic_reconfigure
)

## Declare ROS messages and services
add_message_files(FILES
  Thruster.msg
  Health.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES
  std_msgs
)

## Add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  config/teleop_xbox.config
 # config/simple_pilot.config
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
  message_runtime
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(controller src/controller.cpp)
#target_link_libraries(controller ${catkin_LIBRARIES})
#add_dependencies(controller ${catkin_EXPORTED_TARGETS})

#add_executable(pilot src/pilot.cpp)
#target_link_libraries(pilot ${catkin_LIBRARIES})
#add_dependencies(pilot ${catkin_EXPORTED_TARGETS})

#add_executable(simple_pilot src/simple_pilot.cpp)
#target_link_libraries(simple_pilot ${catkin_LIBRARIES})
#add_dependencies(simple_pilot ${catkin_EXPORTED_TARGETS})

add_executable(teleop_xbox src/teleop_xbox.cpp)
target_link_libraries(teleop_xbox ${catkin_LIBRARIES})
add_dependencies(teleop_xbox ${catkin_EXPORTED_TARGETS})

add_executable(navio_controller src/navio/navio_controller.cpp)
add_library(navio thirdparty/navio/PCA9685.cpp 
                  thirdparty/navio/gpio.cpp thirdparty/navio/I2Cdev.cpp
                  thirdparty/navio/ADS1115.cpp thirdparty/navio/MB85RC04.cpp 
                  thirdparty/navio/MPU9250.cpp thirdparty/navio/MS5611.cpp
                  thirdparty/navio/Ublox.cpp )
target_link_libraries(navio_controller navio ${catkin_LIBRARIES})
add_dependencies(navio_controller ${catkin_EXPORTED_TARGETS})
