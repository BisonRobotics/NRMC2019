cmake_minimum_required(VERSION 2.8.3)
project(vrep_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(FILES DriverMessage.msg)
add_message_files(FILES IMU.msg)
add_service_files(FILES SpawnRobot.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  #LIBRARIES vrep_msgs
  CATKIN_DEPENDS message_runtime
)
