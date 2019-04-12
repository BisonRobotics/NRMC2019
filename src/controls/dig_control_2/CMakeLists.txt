cmake_minimum_required(VERSION 2.8.3)
project(dig_control_2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vesc_access
)

find_package(Boost REQUIRED COMPONENTS system)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  LIBRARIES dig_controller
#  CATKIN_DEPENDS roscpp
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

add_library(dig_controller src/dig_controller.cpp)
target_link_libraries(dig_controller ${catkin_LIBRARIES})

add_executable(dig_teleop_2 src/dig_teleop.cpp)
target_link_libraries(dig_teleop_2 dig_controller ${catkin_LIBRARIES})


#############
## Testing ##
#############
catkin_add_gtest(dig_controller_tests src/tests/dig_controller_tests.cpp)
if (TARGET dig_controller_tests)
    target_link_libraries(dig_controller_tests ${catkin_LIBRARIES} dig_controller gmock_main)
endif ()

