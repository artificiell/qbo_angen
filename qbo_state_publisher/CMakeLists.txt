cmake_minimum_required(VERSION 2.8.3)
project(qbo_state_publisher)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  robot_state_publisher
  urdf
  kdl_parser
  sensor_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp robot_state_publisher urdf kdl_parser sensor_msgs
)

## Build and include 
include_directories(include ${catkin_INCLUDE_DIRS}) 

## Declare a cpp executable
#add_executable(qbo_state_publisher src/qbo_state_publisher.cpp)  
#target_link_libraries(qbo_state_publisher ${catkin_LIBRARIES})
