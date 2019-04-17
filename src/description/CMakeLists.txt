cmake_minimum_required(VERSION 2.8.3)
project(description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf2 tf2_ros geometry_msgs)

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
#  INCLUDE_DIRS include
#  LIBRARIES description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

###########
## Build ##
###########

#include_directories(${catkin_INCLUDE_DIRS})
#add_executable(joint_mappings src/joint_mappings.cpp)
#add_dependencies(joint_mappings ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(joint_mappings ${catkin_LIBRARIES})

#add_executable(tag_location_publisher src/tag_location_publisher.cpp)
#target_link_libraries(tag_location_publisher ${catkin_LIBRARIES})
