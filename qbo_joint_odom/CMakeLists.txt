cmake_minimum_required(VERSION 2.8.3)
project(qbo_joint_odom)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  tf
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf
)

## Build and include 
include_directories(include ${catkin_INCLUDE_DIRS}) 

## Declare a cpp executable
add_executable(qbo_xtion_tf src/qbo_xtion_tf.cpp) 
target_link_libraries(qbo_xtion_tf ${catkin_LIBRARIES})
add_executable(qbo_head_transform src/qbo_head_transform.cpp) 
target_link_libraries(qbo_head_transform ${catkin_LIBRARIES})
