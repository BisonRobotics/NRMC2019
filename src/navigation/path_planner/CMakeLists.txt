cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")

## Non ROS packages
find_package(Boost REQUIRED COMPONENTS thread timer)
find_package(OpenCV 3.3.0)

## ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  occupancy_grid
)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

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

## Build libraries
add_library(path_planner src/path_planner/path_planner.cpp)
target_link_libraries(path_planner ${OpenCV_LIBS})
target_link_libraries(path_planner ${catkin_LIBRARIES})
target_link_libraries(path_planner ${Boost_LIBRARIES})

## Build executables
add_executable(visual_test test/visual_test.cpp)
add_dependencies(visual_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_test ${OpenCV_LIBS})
target_link_libraries(visual_test ${catkin_LIBRARIES})
target_link_libraries(visual_test path_planner)

add_executable(path_planning_example src/examples/path_planning.cpp)
add_dependencies(path_planning_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_example ${OpenCV_LIBS})
target_link_libraries(path_planning_example ${catkin_LIBRARIES})
target_link_libraries(path_planning_example path_planner)