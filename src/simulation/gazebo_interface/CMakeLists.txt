cmake_minimum_required(VERSION 2.8.3)
project(gazebo_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vrep_msgs
  tf
  driver_access
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
#  INCLUDE_DIRS include
#  LIBRARIES gazebo_interface
#  CATKIN_DEPENDS roscpp
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


add_library(driver_plugin SHARED
  src/gazebo_interface_plugin/gazebo_interface_plugin.cpp
  src/gazebo_access_server/gazebo_access_server.cpp
  src/gazebo_drivers/gazebo_wheel_driver.cpp)
add_dependencies(driver_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver_plugin ${GAZEBO_libraries})
target_link_libraries(driver_plugin ${catkin_LIBRARIES})
target_link_libraries(driver_plugin
  boost_system
  boost_filesystem)

