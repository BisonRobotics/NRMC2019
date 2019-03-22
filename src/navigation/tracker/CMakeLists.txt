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
)

find_package(OpenCV 3.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tracker
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  #  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

#if (OpenCV_FOUND)
#  add_definitions(-DOPENCV_ENABLED)
#else()
#  message([WARN] "Unable to find OpenCV, some features will not be enabled")
#endif()

# Camera
add_library(camera_info src/camera/camera_info.cpp)
add_library(ocam_camera src/camera/ocam_camera.cpp)
target_link_libraries(ocam_camera ${ocam_LIBRARIES} camera_info)
add_library(image_camera src/camera/image_camera.cpp)
target_link_libraries(image_camera ${catkin_LIBRARIES} ${OpenCV_LIBS})

# AprilTags
add_library(apriltags_wrapper src/apriltags/apriltags.cpp)
target_link_libraries(apriltags_wrapper ${apriltags_LIBRARIES})

# Tracker node
add_executable(tracker_node src/tracker_node.cpp)
target_link_libraries(tracker_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ocam_camera
        apriltags_wrapper
        camera_info)
