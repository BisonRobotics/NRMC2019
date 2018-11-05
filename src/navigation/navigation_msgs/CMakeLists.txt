cmake_minimum_required(VERSION 2.8.3)
project(navigation_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  actionlib_msgs
  geometry_msgs
)

add_action_files(
  DIRECTORY action
  FILES
    FollowPath.action
)

add_message_files(
  FILES
    Point2D.msg
    BezierGoal.msg
    BezierSegment.msg
    Obstacle.msg
)
add_service_files(
  FILES
    PlanPath.srv
    ObstacleLayout.srv
)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)

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
#  LIBRARIES
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)