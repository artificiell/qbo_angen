cmake_minimum_required(VERSION 2.8.3)
project(qbo_calibration)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  cv_bridge 
  image_transport 
  message_filters
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge image_transport message_filters
)

## Build 
include_directories(include ${catkin_INCLUDE_DIRS}) 

## Declare a cpp executable
add_executable(alignment src/alignment.cpp)
target_link_libraries(alignment ${catkin_LIBRARIES})
