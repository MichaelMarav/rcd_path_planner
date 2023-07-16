cmake_minimum_required(VERSION 3.0.2)
project(apf_planner)


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
 include ${catkin_INCLUDE_DIRS} apf_planner
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/apf.cpp
  include/${PROJECT_NAME}/apf.h
)


add_executable(main src/nodes/todo.cpp src/apf.cpp)
target_link_libraries(main ${apf_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(main ${catkin_EXPORTED_TARGETS})
