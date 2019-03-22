cmake_minimum_required(VERSION 2.8.3)
project(apriltags)

add_compile_options(-std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function)
add_compile_options(-O3 -fno-strict-overflow)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltags
)

include_directories(include)
file(GLOB apriltags_sources src/common/*.c src/tags/*.c src/*.c)
add_library(apriltags ${apriltags_sources})
