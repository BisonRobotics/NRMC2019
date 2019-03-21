cmake_minimum_required(VERSION 2.8.3)
project(teleop)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  vesc_access
)


link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teleop_interface
#  CATKIN_DEPENDS joy rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories (include ${catkin_INCLUDE_DIRS})
include_directories (${GTEST_INCLUDE_DIRS})
include_directories (${vesc_access_INCLUDE_DIRS})

add_library(teleop_interface src/teleop_interface/teleop_interface.cpp)
target_link_libraries (teleop_interface ${catkin_LIBRARIES})

add_executable(teleop src/teleop.cpp)
target_link_libraries (teleop teleop_interface)

