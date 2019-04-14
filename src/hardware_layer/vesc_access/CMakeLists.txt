cmake_minimum_required(VERSION 2.8.3)
project(vesc_access)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vesc_access vesc
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_library(vesc src/vesc_control/vesc_socket_can.cpp)
target_link_libraries(vesc ${catkin_LIBRARIES})

add_library(vesc_access src/vesc_access/vesc_access.cpp)
target_link_libraries(vesc_access ${catkin_LIBRARIES} vesc)

#add_executable(vesc_access_integration_test test/vesc_access_integration_test.cpp)
#target_link_libraries(vesc_socket_can_integration_test vesc_lib)

#add_executable (vesc_socket_can_integration_test test/vesc_socket_can_integration_test.cpp)
#target_link_libraries(vesc_access_integration_test vesc_access)

#add_executable(vesc_access_pot_limit_test test/vesc_access_pot_limit_test.cpp)
#target_link_libraries(vesc_access_pot_limit_test vesc_access)

#add_executable(vesc_socket_pot_test test/vesc_socket_pot_test.cpp)
#target_link_libraries(vesc_socket_pot_test vesc_lib ${catkin_LIBRARIES})

#add_executable(runtime_check test/vesc_runtime_check.cpp)
#target_link_libraries(runtime_check vesc_lib)

#############
## Testing ##
#############
catkin_add_gtest(test_vesc_access src/vesc_access/tests/vesc_access_tests.cpp)
if(TARGET test_vesc_access)
  target_link_libraries(
    test_vesc_access
    ${catkin_LIBRARIES}
    vesc_access
    gmock_main)
endif()