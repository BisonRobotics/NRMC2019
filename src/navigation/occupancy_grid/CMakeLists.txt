cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Boost
find_package(Boost REQUIRED COMPONENTS thread)

## ROS Stuff
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## GMock
link_directories(/usr/src/gmock/build)
set(gtest_LIBRARIES gmock_main)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES julia_test
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

find_package(OpenCV 3.3.0)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

## Build libraries
add_library(occupancy_grid src/occupancy_grid/occupancy_grid.cpp)
target_link_libraries(occupancy_grid ${OpenCV_LIBS})
target_link_libraries(occupancy_grid ${catkin_LIBRARIES})

## Build executable
add_executable(drawing_test src/drawing_test.cpp)
target_link_libraries(drawing_test ${OpenCV_LIBS})
target_link_libraries(drawing_test ${catkin_LIBRARIES})
target_link_libraries(drawing_test occupancy_grid)


#target_link_libraries(path_planner_node julia.so.0.7.0)
#add_dependencies(julia_test julia_test_generate_messages_cpp)

## Tests
catkin_add_gtest(occupancy_grid_test test/occupancy_grid/occupancy_grid_test.cpp)
if(TARGET occupancy_grid_test)
    target_link_libraries(occupancy_grid_test ${OpenCV_LIBS})
    target_link_libraries(occupancy_grid_test occupancy_grid)
endif()

catkin_add_gtest(geometry_test test/occupancy_grid/geometry_test.cpp)
if(TARGET geometry_test)
    target_link_libraries(geometry_test ${OpenCV_LIBS})
endif()
