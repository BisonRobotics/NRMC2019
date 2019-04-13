cmake_minimum_required(VERSION 3.0.2)
project(dig_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs 
  actionlib)

link_directories (/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES Outrigger.action Dig.action Dump.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs message_generation
)
