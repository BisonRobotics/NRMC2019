cmake_minimum_required(VERSION 2.8.3)
project(vive_access)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread timer)


catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES vive_access
  CATKIN_DEPENDS roscpp
  #DEPENDS system_lib
)


include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)
include_directories({catkin_INCLUDE_DIRS})

link_directories(/usr/local/lib/)

add_library(lighthouse_tracking src/lighthouse_tracking.cpp)
target_link_libraries(lighthouse_tracking openvr_api)
target_link_libraries(lighthouse_tracking dl)

add_executable(vive_test src/vive_test.cpp)
target_link_libraries(vive_test lighthouse_tracking)
target_link_libraries(vive_test ${Boost_LIBRARIES})

