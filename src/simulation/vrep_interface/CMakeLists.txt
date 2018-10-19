cmake_minimum_required(VERSION 2.8.3)
project(vrep_interface)
add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  vrep_msgs
  driver_access
  vrep_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# GMock
link_directories(/usr/src/gmock/build)
set(gtest_LIBRARIES gmock_main)

# Define package
catkin_package(
  CATKIN_DEPENDS
    roslib
    rostime
    rosconsole
    rospack
    roscpp
    roscpp_serialization
    tf
    vrep_msgs
  DEPENDS
    Boost
)


add_library(vrep_driver src/vrep_drivers/vrep_driver_base.cpp)
add_dependencies(vrep_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_driver ${catkin_LIBRARIES})

#add_library(vrep_wheel_driver src/vrep_drivers/vrep_wheel_driver.cpp)
#add_dependencies(vrep_wheel_driver ${catkin_EXPORTED_TARGETS})
#target_link_libraries(vrep_wheel_driver ${catkin_LIBRARIES})
#target_link_libraries(vrep_wheel_driver vrep_driver)

add_library(ros_server
  src/vrep_library/v_repLib.cpp
  src/vrep_interface/ros_server.cpp)
add_dependencies(ros_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_server ${catkin_LIBARIES})
target_link_libraries(ros_server boost_system)

add_library(v_repExtRosInterface
  src/vrep_library/v_repLib.cpp
  src/vrep_interface/vrep_plugin.cpp
  src/vrep_interface/vrep_interface.cpp
  src/vrep_library/v_repLib.cpp)
target_link_libraries(v_repExtRosInterface ros_server)
#target_link_libraries(v_repExtRosInterface pthread)
#target_link_libraries(v_repExtRosInterface dl)
