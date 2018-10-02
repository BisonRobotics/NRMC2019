cmake_minimum_required(VERSION 2.8.3)
project(drive_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  vesc_access
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES robot_control
        CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${vesc_access_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/NRMC2018/devel/include)

## Build libraries
add_library(drive_controller
        src/drive_controller/drive_controller.cpp)
add_dependencies(drive_controller vesc_access ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_controller ${catkin_LIBRARIES} vesc_access)

#############
## Install ##
#############


#############
## Testing ##
#############

catkin_add_gtest(test_drive_controller include/drive_controller/drive_controller.h src/drive_controller/test/drive_controller_tests.cpp)
if(TARGET test_drive_controller)
  target_link_libraries(test_drive_controller
    ${catkin_LIBRARIES}
    drive_controller
    gmock_main
  )
  add_dependencies(test_drive_controller vesc_access)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} vesc_access)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
