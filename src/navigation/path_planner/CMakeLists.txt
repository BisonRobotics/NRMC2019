cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Non ROS packages
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV 3.3.0)

## ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  occupancy_grid
)
include_directories(include ${catkin_INCLUDE_DIRS})

## GMock
link_directories(/usr/src/gmock/build)
set(gtest_LIBRARIES gmock_main)

## Define package
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES occupancy_grid
  CATKIN_DEPENDS roscpp std_msgs occupancy_grid
  DEPENDS OpenCV Boost
)

## Build executables
add_executable(path_planner_test src/visual_test.cpp)
add_dependencies(path_planner_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_test ${OpenCV_LIBS})
target_link_libraries(path_planner_test ${catkin_LIBRARIES})
