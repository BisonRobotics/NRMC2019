cmake_minimum_required(VERSION 2.8.3)
project(stepper)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

find_package(Boost COMPONENTS program_options timer REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
  LIBRARIES stepper
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stepper src/stepper.cpp)
target_link_libraries(stepper ${catkin_LIBRARIES})

add_executable(stepper_node src/stepper_node.cpp)
target_link_libraries(stepper_node ${catkin_LIBRARIES} stepper)

add_executable(stepper_tool src/stepper_tool.cpp)
target_link_libraries(stepper_tool ${catkin_LIBRARIES} stepper)
target_link_libraries(stepper_tool ${Boost_LIBRARIES})

catkin_add_gtest(stepper_tests src/stepper.cpp src/tests/stepper_tests.cpp)
if(TARGET stepper_tests)
    target_link_libraries(stepper_tests gmock_main ${catkin_LIBRARIES})
endif()
