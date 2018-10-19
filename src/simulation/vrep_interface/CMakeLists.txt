cmake_minimum_required(VERSION 2.8.3)
project(vrep_interface)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  vrep_msgs
  driver_access
)

catkin_package(CATKIN_DEPENDS
  roslib
  rostime
  rosconsole
  rospack
  roscpp
  roscpp_serialization
  tf
  vrep_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(vrep_driver src/vrep_drivers/vrep_driver_base.cpp)
add_dependencies(vrep_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_driver ${catkin_LIBRARIES})

add_library(vrep_wheel_driver src/vrep_drivers/vrep_wheel_driver.cpp)
add_dependencies(vrep_wheel_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_wheel_driver ${catkin_LIBRARIES})
target_link_libraries(vrep_wheel_driver vrep_driver)

add_library(vrep_server
  src/vrep_interface/vrep_server.cpp
  src/vrep_robot/vrep_robot.cpp)
add_dependencies(vrep_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_server
  pthread
  dl
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES}
  vrep_wheel_driver)

add_library(v_repExtRosInterface
  src/vrep_interface/vrep_plugin.cpp
  src/vrep_library/v_repLib.cpp
  src/vrep_interface/vrep_interface.cpp)
add_dependencies(v_repExtRosInterface ${catkin_EXPORTED_TARGETS})
target_link_libraries(v_repExtRosInterface
  pthread
  dl
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES}
  vrep_wheel_driver
  vrep_server)
