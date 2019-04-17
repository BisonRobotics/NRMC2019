cmake_minimum_required(VERSION 2.8.3)
project(tracker_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_transport
        image_transport
        std_msgs
        tracker)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Qt5Core QUIET)
find_package(Qt5Gui QUIET)
find_package(Qt5Widgets QUIET)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracker_gui
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Based off of example: https://github.com/ros-visualization/rqt_common_plugins/blob/...
# 619481f13084db01c7e2233b754bb0409ce1c44a/rqt_image_view/CMakeLists.txt
if(DEFINED ENV{ON_ROBOT})
    message(AUTHOR_WARNING " ON_ROBOT set, not building visuals or simulation")
else()
if (${Qt5Core_FOUND} AND ${Qt5Gui_FOUND} AND ${Qt5Widgets_FOUND})
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    qt5_wrap_cpp(tracker_gui_MOC include/tracker_gui/tracker_gui.h)
    qt5_wrap_ui(tracker_gui_UI ui/tracker_gui.ui)
    add_executable(tracker_gui
            src/main.cpp
            src/tracker_gui.cpp
            ${tracker_gui_MOC}
            ${tracker_gui_UI})
    qt5_use_modules(tracker_gui Core Gui Widgets)
    target_link_libraries(tracker_gui
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS})
    add_dependencies(tracker_gui ${catkin_EXPORTED_TARGETS})
else()
  message(AUTHOR_WARNING "The qt5 library is not available, tracker_gui will not be built")
endif()
endif()
