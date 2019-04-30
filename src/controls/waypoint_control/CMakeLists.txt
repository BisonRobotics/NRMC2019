cmake_minimum_required(VERSION 2.8.3)
project(waypoint_control)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vesc_access
  geometry_msgs
  visualization_msgs
  tf2
  nav_msgs
  navigation_msgs
  actionlib
  actionlib_msgs
  navigation_msgs
  joy
  visualization_msgs
  interactive_markers
  utilities
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_action_files(
  DIRECTORY action
  FILES
  WaypointControl.action
)

add_message_files(FILES FeedbackInfo.msg Command.msg Waypoint.msg Debug.msg)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs navigation_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_control_client waypoint_control_config teleop_interface
  CATKIN_DEPENDS roscpp std_msgs navigation_msgs vesc_access
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_library(teleop_interface src/teleop_interface/teleop_interface.cpp)
target_link_libraries(teleop_interface ${catkin_LIBRARIES})

add_library(waypoint_control_config src/waypoint_control/config.cpp)
add_dependencies(waypoint_control_config ${waypoint_control_EXPORTED_TARGETS})
target_link_libraries(waypoint_control_config ${catkin_LIBRARIES})

add_library(waypoint_controller src/waypoint_control/waypoint_controller.cpp src/waypoint_control/feedback.cpp)
add_dependencies(waypoint_controller ${waypoint_control_EXPORTED_TARGETS})
target_link_libraries(waypoint_controller waypoint_control_config ${catkin_LIBRARIES})

add_library(waypoint_control_client src/waypoint_control/waypoint_control_client.cpp)
add_dependencies(waypoint_control_client ${waypoint_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(waypoint_control_client waypoint_control_config ${catkin_LIBRARIES})

add_executable(teleop src/teleop_interface/teleop.cpp)
target_link_libraries(teleop teleop_interface ${catkin_LIBRARIES})

add_executable(waypoint_control_server src/waypoint_control/waypoint_control_server.cpp)
add_dependencies(waypoint_control_server ${waypoint_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(waypoint_control_server waypoint_control_config waypoint_controller ${catkin_LIBRARIES})

#add_executable(waypoint_control_client_node src/waypoint_control/waypoint_control_client_node.cpp)
#add_dependencies(waypoint_control_client_node ${waypoint_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
#target_link_libraries(waypoint_control_client_node waypoint_control_config waypoint_control_client ${catkin_LIBRARIES})


if(DEFINED ENV{ON_ROBOT})
  message(AUTHOR_WARNING " ON_ROBOT set, not building visuals or simulation")
else()

endif()

#############
## Testing ##
#############
catkin_add_gtest(teleop_interface_tests src/teleop_interface/tests/teleop_interface_tests.cpp)
if(TARGET teleop_interface_tests)
  target_link_libraries(teleop_interface_tests  teleop_interface ${catkin_LIBRARIES} gmock_main)
endif()

catkin_add_gtest(waypoint_control_tests src/waypoint_control/tests/waypoint_control_tests.cpp)
if(TARGET waypoint_control_tests)
  target_link_libraries(waypoint_control_tests waypoint_controller ${catkin_LIBRARIES} gmock_main)
  add_dependencies(waypoint_control_tests ${waypoint_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
endif()
