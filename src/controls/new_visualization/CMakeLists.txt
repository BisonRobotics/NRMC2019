cmake_minimum_required(VERSION 2.8.3)
project(drive_controller_vis)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  navigation_msgs
  rqt_gui
  rqt_gui_cpp
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

#if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
#  find_package(Qt5Widgets REQUIRED)
#else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
#endif()

find_path(
  QWT_INCLUDE_DIRS
  NAMES
    qwt_plot.h
  PATHS
    /usr/include/qwt-qt4
    /usr/include/qwt
    /usr/local/include/qwt
)

#if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
#  find_library(
#    QWT_LIBRARIES
#    NAMES
#      qwt-qt5
#  )
#else()
  find_library(
    QWT_LIBRARIES
    NAMES
      qwt-qt4
      qwt
  )
#endif()

catkin_package(
        INCLUDE_DIRS include

        CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs navigation_msgs
)

#what are these?
#qt5_wrap_cpp(rqt_multiplot_MOCS ${rqt_multiplot_HDRS})
#qt5_wrap_ui(rqt_multiplot_UIS_H ${rqt_multiplot_UIS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIRS} $ENV{HOME}/NRMC2018/devel/include)

## Build libraries
add_library(drive_controller_vis_window
            src/drive_controller_vis/window.cpp)
add_dependencies(drive_controller_vis_window ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_controller_vis_window ${catkin_LIBRARIES}  ${QWT_LIBRARIES} )
target_link_libraries(drive_controller_vis_window ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

#############
## Install ##
#############

add_executable(drive_controller_vis_master src/drive_controller_vis_master.cpp)
add_dependencies(drive_controller_vis_master drive_controller_vis_window ${catkin_EXPORTED_TARGETS}) #navigation_msgs localization
target_link_libraries (drive_controller_vis_master 
  drive_controller_vis_window 
  ${QWT_LIBRARIES} 
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${catkin_LIBRARIES} 
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} vesc_access)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
