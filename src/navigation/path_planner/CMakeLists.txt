cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Non ROS packages
find_package(Boost REQUIRED COMPONENTS thread timer)
find_package(OpenCV 3.2.0)

# ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  navigation_msgs
  occupancy_grid
  actionlib
)
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# GMock
link_directories(/usr/src/gmock/build)
set(gtest_LIBRARIES gmock_main)

# Define package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner
  CATKIN_DEPENDS
    roscpp
    std_msgs
    navigation_msgs
    occupancy_grid
  DEPENDS
    OpenCV
    Boost
)

# Build libraries
add_library(path_planner src/path_planner/path_planner.cpp)
target_link_libraries(path_planner ${OpenCV_LIBS})
target_link_libraries(path_planner ${catkin_LIBRARIES})
target_link_libraries(path_planner ${Boost_LIBRARIES})

# Build executables
if(DEFINED ENV{ON_ROBOT})
  message(AUTHOR_WARNING " ON_ROBOT set, not building visuals or simulation")
else()
#  add_executable(visual_test test/visual_test.cpp)
#  add_dependencies(visual_test ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(visual_test ${OpenCV_LIBS})
#  target_link_libraries(visual_test ${catkin_LIBRARIES})
#  target_link_libraries(visual_test path_planner)
#
#  add_executable(path_planning_example src/examples/path_planning.cpp)
#  add_dependencies(path_planning_example ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(path_planning_example ${OpenCV_LIBS})
#  target_link_libraries(path_planning_example ${catkin_LIBRARIES})
#  target_link_libraries(path_planning_example path_planner)
#
#  add_executable(path_planner_node src/path_planner_node/path_planner_node.cpp)
#  add_dependencies(path_planner_node ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(path_planner_node ${OpenCV_LIBS})
#  target_link_libraries(path_planner_node ${catkin_LIBRARIES})
#  target_link_libraries(path_planner_node path_planner)
#
#  add_executable(action_server_example src/path_planner_node/action_server_example.cpp)
#  add_dependencies(action_server_example ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(action_server_example ${catkin_LIBRARIES})
#  target_link_libraries(action_server_example path_planner)
endif()

