cmake_minimum_required(VERSION 2.8.3)
project(tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  apriltags
  ocam
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_generation
  genmsg
  actionlib_msgs
  stepper
)

find_package(OpenCV 3.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS timer REQUIRED)

add_message_files(FILES
    Debug.msg
)

add_service_files(FILES
    GetUInt.srv
)

add_action_files(DIRECTORY action FILES
    SetUInt.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tracker
  CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        stepper
  #  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

# Add gmock
link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

# Camera
add_library(camera_info src/camera/camera_info.cpp)
add_library(ocam_camera src/camera/ocam_camera.cpp)
target_link_libraries(ocam_camera ${ocam_LIBRARIES} camera_info)
add_library(image_camera src/camera/image_camera.cpp)
target_link_libraries(image_camera ${catkin_LIBRARIES} ${OpenCV_LIBS})


# Detector
add_library(detector
  src/detector.cpp
  src/tag.cpp
  src/util.cpp)
target_link_libraries(detector
        ${apriltags_LIBRARIES}
        ${catkin_LIBRARIES})

# Tracker node
add_executable(tracker_node
  src/thread.cpp
  src/tracker_node.cpp)
target_link_libraries(tracker_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ocam_camera
        detector
        camera_info)
add_dependencies(tracker_node tracker_gencpp)

catkin_add_gtest(tag_tests src/tag.cpp src/tests/tag_tests.cpp)
if(TARGET tag_tests)
    target_link_libraries(tag_tests gmock_main ${catkin_LIBRARIES})
endif()