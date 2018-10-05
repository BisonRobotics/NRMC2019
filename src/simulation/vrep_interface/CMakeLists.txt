cmake_minimum_required(VERSION 2.8.3)
project(vrep_interface)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  dynamic_reconfigure
  vrep_msg_and_srv
  wheel_control
)

catkin_package(CATKIN_DEPENDS
  roslib
  rosconsole
  rostime
  rosconsole
  rospack
  roscpp
  roscpp_serialization
  tf
  vrep_msg_and_srv
  wheel_control
)

include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/NRMC2018/devel/include)

add_library(v_repExtRosInterface
  src/vrep_interface_plugin/vrep_interface_plugin.cpp
  src/vrep_lib/v_repLib.cpp
  src/ros_interface_server/ros_server.cpp
  src/obstacle_field/obstacle_field.cpp
  src/vrep_robot/vrep_robot.cpp
  src/vrep_wheels/vrep_wheels.cpp)

add_dependencies(v_repExtRosInterface ${catkin_EXPORTED_TARGETS})

target_link_libraries(v_repExtRosInterface
  pthread
  dl
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES}
)
