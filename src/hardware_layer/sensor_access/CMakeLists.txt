cmake_minimum_required(VERSION 2.8.3)
project(sensor_access)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  vrep_msgs
)

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
  INCLUDE_DIRS include/
  LIBRARIES lpresearch_imu apriltag_tracker_interface vrep_imu_access
  CATKIN_DEPENDS vrep_msgs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include ${catkin_INCLUDE_DIRS}
)

add_library (vrep_imu_access src/vrep_access/vrep_imu.cpp)
add_dependencies(vrep_imu_access ${catkin_EXPORTED_TARGETS})
target_link_libraries (vrep_imu_access ${catkin_LIBRARIES})

add_library (vrep_position_access src/vrep_access/vrep_position_sensor.cpp)
add_dependencies(vrep_position_access ${catkin_EXPORTED_TARGETS})
target_link_libraries (vrep_position_access ${catkin_LIBRARIES})

add_library (lpresearch_imu src/lp_research/lpresearchimu.cpp)
target_link_libraries (lpresearch_imu ${catkin_LIBRARIES})
#add_executable (lpresearch_integration_test tests/lpresearchtest.cpp)
#target_link_libraries (lpresearch_integration_test lpresearch_imu ${catkin_LIBRARIES})

add_library (apriltag_tracker_interface src/apriltag_tracker_interface/apriltag_tracker_interface.cpp)
target_link_libraries (apriltag_tracker_interface ${catkin_LIBRARIES})

#add_executable(apriltag_integration_test tests/apriltag_integration.cpp)
#target_link_libraries(apriltag_integration_test apriltag_tracker_interface ${catkin_LIBRARIES})


if(DEFINED ENV{ON_ROBOT})
 message(AUTHOR_WARNING " ON_ROBOT set, not building visuals or simulation")
else()
 add_executable(vrep_imu_test_node src/vrep_access/vrep_imu_test_node.cpp)
 target_link_libraries(vrep_imu_test_node vrep_imu_access ${catkin_LIBRARIES})
 add_executable(vrep_position_test_node src/vrep_access/vrep_position_test_node.cpp)
 target_link_libraries(vrep_position_test_node vrep_position_access ${catkin_LIBRARIES})
endif()

