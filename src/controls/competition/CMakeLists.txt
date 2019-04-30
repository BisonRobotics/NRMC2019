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
  utilities
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES Time.msg Debug.msg)
generate_messages(DEPENDENCIES std_msgs)


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

add_library(waypoint_visuals src/waypoint_visuals.cpp)
add_dependencies(waypoint_visuals ${waypoint_control_EXPORTED_TARGETS} ${visualization_msgs_EXPORTED_TARGETS})
target_link_libraries(waypoint_visuals ${catkin_LIBRARIES})

add_library(competition_config src/config.cpp)
add_dependencies(competition_config ${waypoint_control_EXPORTED_TARGETS} ${visualization_msgs_EXPORTED_TARGETS})
target_link_libraries(competition_config ${catkin_LIBRARIES})

add_library(competition_controller src/competition_controller.cpp)
add_dependencies(competition_controller ${competition_EXPORTED_TARGETS} ${waypoint_control_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(competition_controller competition_config waypoint_visuals ${catkin_LIBRARIES})

add_executable(competition_node src/competition_node.cpp)
add_dependencies(competition_node ${competition_EXPORTED_TARGETS} ${waypoint_control_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(competition_node competition_controller ${catkin_LIBRARIES})

add_executable(visuals_test_node src/visuals_test_node.cpp)
add_dependencies(visuals_test_node ${competition_EXPORTED_TARGETS} ${waypoint_control_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(visuals_test_node waypoint_visuals ${catkin_LIBRARIES})

add_executable(config_test_node src/config_test_node.cpp)
add_dependencies(config_test_node ${competition_EXPORTED_TARGETS} ${waypoint_control_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(config_test_node competition_config ${catkin_LIBRARIES})
