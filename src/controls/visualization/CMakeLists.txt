cmake_minimum_required(VERSION 2.8.3)
project(drive_controller_visualization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  navigation_msgs
)

link_directories(/usr/src/gmock/build)

catkin_python_setup()

catkin_package(
        CATKIN_DEPENDS rospy std_msgs navigation_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


## Build libraries


#############
## Install ##
#############

#add_executable(drive_controller_visualization scripts/drive_controller_plot.py)

catkin_install_python(PROGRAMS scripts/drive_controller_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############


