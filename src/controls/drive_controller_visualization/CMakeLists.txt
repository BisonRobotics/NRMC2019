cmake_minimum_required(VERSION 2.8.3)
project(drive_controller_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2
  nav_msgs
  navigation_msgs
  cv_bridge
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY ../../navigation/navigation_msgs/action FILES FollowPath.action)
generate_messages(DEPENDENCIES actionlib_msgs navigation_msgs)


catkin_package(
        INCLUDE_DIRS include

        CATKIN_DEPENDS roscpp std_msgs cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} $ENV{HOME}/NRMC2018/devel/include)

## Build libraries
#add_library(drive_controller_visualization
#        src/drive_controller_visualization/drive_controller_visualization.cpp)
#add_dependencies(drive_controller vesc_access) # ${catkin_EXPORTED_TARGETS})
#target_link_libraries(drive_controller ${catkin_LIBRARIES} vesc_access)

#############
## Install ##
#############

add_executable(drive_controller_visualization_master src/drive_controller_visualization_master.cpp)
#add_dependencies(drive_controller_master drive_controller vrep_driver_access ros_driver_access driver_access sim_robot ${catkin_EXPORTED_TARGETS}) #navigation_msgs localization
target_link_libraries (drive_controller_visualization_master 
 ${OpenCV_LIBS} 
 ${catkin_LIBRARIES} 
)



#############
## Testing ##
#############

#catkin_add_gtest(test_drive_controller include/drive_controller/drive_controller.h src/drive_controller/test/drive_controller_tests.cpp)
#if(TARGET test_drive_controller)
#  target_link_libraries(test_drive_controller
#    ${catkin_LIBRARIES}
#    drive_controller
#    gmock_main
#  )
#  add_dependencies(test_drive_controller vesc_access)
#endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} vesc_access)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
