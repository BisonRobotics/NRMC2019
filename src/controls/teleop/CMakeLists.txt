cmake_minimum_required(VERSION 2.8.3)
project(teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  driver_access
)

find_package(Qt5Core QUIET)
find_package(Qt5Gui QUIET)
find_package(Qt5Widgets QUIET)


link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teleop
  CATKIN_DEPENDS driver_access
  #DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(teleop src/teleop/teleop.cpp)
target_link_libraries (teleop ${catkin_LIBRARIES})

add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries (teleop_node teleop)

# Based off of example: https://github.com/ros-visualization/rqt_common_plugins/blob/...
# 619481f13084db01c7e2233b754bb0409ce1c44a/rqt_image_view/CMakeLists.txt
#if (${Qt5Core_FOUND} AND ${Qt5Gui_FOUND} AND ${Qt5Widgets_FOUND})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  qt5_wrap_cpp(qt_joystick_MOC include/qt_joystick/main_window.h)
  qt5_wrap_ui(qt_joystick_UI ui/qt_joystick/main_window.ui)
  add_executable(qt_joystick
    src/qt_joystick/main.cpp
    src/qt_joystick/main_window.cpp
    ${qt_joystick_MOC}
    ${qt_joystick_UI})
  qt5_use_modules(qt_joystick Core Gui Widgets)
  target_link_libraries(qt_joystick ${catkin_LIBRARIES})
#else()
#  message(AUTHOR_WARNING "The qt5 library is not available, qt_joystick will not be built")
#endif()