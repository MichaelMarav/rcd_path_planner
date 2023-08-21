cmake_minimum_required(VERSION 3.0.2)
project(motion_planner)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES apf_planner
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(
 include ${catkin_INCLUDE_DIRS} 
)
