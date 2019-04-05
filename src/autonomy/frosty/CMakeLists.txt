cmake_minimum_required(VERSION 2.8.3)
project(frosty_autonomy)

add_compile_options(-std=c++11)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  navigation_msgs
  geometry_msgs
  vrep_msgs
  std_srvs
  actionlib
  actionlib_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp actionlib actionlib_msgs std_msgs navigation_msgs geometry_msgs vrep_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Build libraries
add_library(frosty_state_machine src/frosty_state_machine/frosty_state_machine.cpp)
add_dependencies(frosty_state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(frosty_state_machine ${catkin_LIBRARIES})

# Define executable not using any messages/services provided by this package
add_executable(frosty src/frosty.cpp)
add_dependencies(frosty frosty_state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(frosty frosty_state_machine ${catkin_LIBRARIES})