cmake_minimum_required(VERSION 2.8.3)
project(dig_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vesc_access
  std_msgs
  sensor_msgs
  actionlib_msgs
  actionlib
  utilities
)

find_package(Boost REQUIRED COMPONENTS system)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

add_message_files(
  FILES
  StateString.msg
  State.msg
  DriveValues.msg
  DrivePositions.msg
  Debug.msg
)

add_action_files(
  DIRECTORY action
  FILES DigControl.action Dump.action Dig.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

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
  LIBRARIES dig_controller dig_control_client
  CATKIN_DEPENDS roscpp message_generation actionlib_msgs
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

add_library(dig_control_config src/config.cpp)
target_link_libraries(dig_control_config ${catkin_LIBRARIES})
add_dependencies(dig_control_config ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(dig_controller src/dig_controller.cpp)
target_link_libraries(dig_controller dig_control_config ${catkin_LIBRARIES})
add_dependencies(dig_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(dig_control_server src/dig_control_server.cpp)
target_link_libraries(dig_control_server dig_control_config dig_controller ${catkin_LIBRARIES})
add_dependencies(dig_control_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(dig_control_client src/dig_control_client.cpp)
target_link_libraries(dig_control_client ${catkin_LIBRARIES})
add_dependencies(dig_control_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(dig_control_client_node src/dig_control_client_node.cpp)
target_link_libraries(dig_control_client_node dig_control_client dig_controller ${catkin_LIBRARIES})
add_dependencies(dig_control_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Testing ##
#############
catkin_add_gtest(dig_controller_tests src/tests/dig_controller_tests.cpp)
if (TARGET dig_controller_tests)
  target_link_libraries(dig_controller_tests ${catkin_LIBRARIES} dig_controller gmock_main)
endif ()

