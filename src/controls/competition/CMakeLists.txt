cmake_minimum_required(VERSION 2.8.3)
project(competition)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  roscpp
  std_msgs
  sensor_msgs
  navigation_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
  dig_control
  waypoint_control
  path_planner
  occupancy_grid
  utilities
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
  LIBRARIES joy
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

add_library(bezier_visuals src/bezier_visuals.cpp)
add_dependencies(bezier_visuals ${visualization_msgs_EXPORTED_TARGETS})
target_link_libraries(bezier_visuals ${catkin_LIBRARIES})

add_library(controller src/controller.cpp)
add_dependencies(controller ${competition_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(controller joy ${catkin_LIBRARIES})

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${competition_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(controller_node controller ${catkin_LIBRARIES})