cmake_minimum_required(VERSION 2.8.3)
project(drive_controller)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vesc_access
  driver_access
  localization
  geometry_msgs
  visualization_msgs
  tf2
  nav_msgs
  navigation_msgs
  occupancy_grid
  actionlib
  actionlib_msgs
  navigation_msgs
  cv_bridge
  joy
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

find_package(OpenCV REQUIRED)

add_action_files(
  DIRECTORY action
  FILES
  DriveControl.action
)

add_message_files(FILES
  WheelStates.msg
  ErrorStates.msg
  PathInfo.msg)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs navigation_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS driver_access roscpp std_msgs navigation_msgs localization cv_bridge
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

add_library(drive_controller src/drive_controller/drive_controller.cpp)
add_dependencies(drive_controller ${drive_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries(drive_controller ${catkin_LIBRARIES})

add_executable(drive_controller_master src/drive_controller/drive_controller_master.cpp)
add_dependencies(drive_controller_master ${drive_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries (drive_controller_master drive_controller ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(drive_control_server src/drive_controller/drive_control_server.cpp)
add_dependencies(drive_control_server ${drive_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
target_link_libraries (drive_control_server drive_controller teleop_interface ${catkin_LIBRARIES})

add_library(teleop_interface src/teleop_interface/teleop_interface.cpp)
target_link_libraries (teleop_interface ${catkin_LIBRARIES})

add_executable(teleop src/teleop_interface/teleop.cpp)
target_link_libraries (teleop teleop_interface ${catkin_LIBRARIES})

if(DEFINED ENV{ON_ROBOT})
  message(AUTHOR_WARNING " ON_ROBOT set, not building visuals or simulation")
else()
  add_library(drive_controller_visualization_graph src/drive_controller_visualization/dcvis_multiplot.cpp)
  add_dependencies(drive_controller ${drive_controller_EXPORTED_TARGETS} ${navigation_msgs_EXPORTED_TARGETS})
  target_link_libraries(drive_controller_visualization_graph ${OpenCV_LIBS})
  add_executable(drive_controller_visualization_master
    src/drive_controller_visualization/drive_controller_visualization_master.cpp)
  add_dependencies(drive_controller_visualization_master ${drive_controller_EXPORTED_TARGETS})
  target_link_libraries (drive_controller_visualization_master
    drive_controller_visualization_graph
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})
endif()

#############
## Testing ##
#############

catkin_add_gtest(test_drive_controller src/drive_controller/test/drive_controller_tests.cpp)
if(TARGET test_drive_controller)
  target_link_libraries(test_drive_controller drive_controller ${catkin_LIBRARIES} gmock_main)
endif()

catkin_add_gtest(teleop_interface_tests src/teleop_interface/tests/teleop_interface_tests.cpp)
if(TARGET teleop_interface_tests)
  target_link_libraries(teleop_interface_tests  teleop_interface ${catkin_LIBRARIES} gmock_main)
endif()
