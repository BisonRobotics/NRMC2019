cmake_minimum_required(VERSION 2.8.3)
project(vrep_plugin)

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

add_library(vrep_interface src/vrep_plugin/interface.cpp)
add_dependencies(vrep_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_interface ${catkin_LIBRARIES})

add_library(vrep_drivers src/vrep_plugin/driver_base.cpp src/vrep_plugin/driver_wheel.cpp)
add_dependencies(vrep_drivers ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_drivers ${catkin_LIBRARIES})

add_library(vrep_server src/vrep_plugin/server.cpp src/vrep_plugin/robot.cpp)
add_dependencies(vrep_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_server
  vrep_drivers
  vrep_interface
  pthread
  dl
  boost_system
  boost_filesystem
  ${catkin_LIBRARIES})

add_library(v_repExtNRMC2019Plugin src/vrep_plugin/plugin.cpp src/vrep_library/v_repLib.cpp)
add_dependencies(v_repExtNRMC2019Plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(v_repExtNRMC2019Plugin
  pthread
  dl
  boost_system
  boost_filesystem
  vrep_server)
