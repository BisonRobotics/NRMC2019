cmake_minimum_required(VERSION 2.8.3)
project(driver_access)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Add gmock
link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

find_package(catkin REQUIRED
  vrep_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES driver_access_base vrep_driver_access
  CATKIN_DEPENDS vrep_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(driver_access_base src/driver_access/driver_access_base.cpp)
add_library(vrep_driver_access src/vrep_driver_access/vrep_driver_access.cpp)
add_dependencies(vrep_driver_access ${catkin_EXPORTED_TARGETS})
target_link_libraries(vrep_driver_access driver_access_base)


catkin_add_gtest(driver_access_tests tests/driver_access/driver_access_tests.cpp)
if (TARGET driver_access_tests)
  target_link_libraries(driver_access_tests ${catkin_LIBRARIES})
  target_link_libraries(driver_access_tests gmock_main)
  target_link_libraries(driver_access_tests driver_access_base)
endif ()
