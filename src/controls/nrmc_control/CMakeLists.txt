cmake_minimum_required(VERSION 2.8.3)
project(competition)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  std_msgs
  sensor_msgs
  navigation_msgs
  drive_controller
  dig_control
  path_planner
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES competition
  CATKIN_DEPENDS actionlib roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(joy src/joy.cpp)
add_dependencies(joy ${sensor_msgs_EXPORTED_TARGETS})
target_link_libraries(joy ${catkin_LIBRARIES})

add_library(controller src/controller.cpp)
add_dependencies(controller ${competition_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${competition_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(controller_node controller ${catkin_LIBRARIES})