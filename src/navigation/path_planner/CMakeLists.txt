cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Julia stuff
include_directories(/opt/julia/include/julia)
link_directories(/opt/julia/lib/julia /opt/julia/lib)
add_definitions(-DJULIA_ENABLE_THREADING=1)
set(julia_deps /opt/julia/lib/julia)

## Boost
find_package(Boost REQUIRED COMPONENTS thread)

## ROS Stuff
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES julia_test
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Build executable
add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})
target_link_libraries(path_planner_node julia)
target_link_libraries(path_planner_node ${julia_deps}/libstdc++.so.6)
#target_link_libraries(path_planner_node julia.so.0.7.0)
#add_dependencies(julia_test julia_test_generate_messages_cpp)
