cmake_minimum_required(VERSION 2.8.3)
project(pc2cm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp 
)

include_directories (include ${catkin_INCLUDE_DIRS} )
include_directories(${GTEST_INCLUDE_DIRS})

add_library(pc2_processor src/pc2_processor/pc2_processor.cpp)

catkin_add_gtest(pc2_processor_tests include/pc2_processor/pc2_processor.h src/pc2_processor/test/pc2_processor_tests.cpp)
if(TARGET pc2_processor_tests)
  target_link_libraries(pc2_processor_tests
    ${catkin_LIBRARIES}
    gmock_main
    pc2_processor
  )
endif()