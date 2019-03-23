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
)

#find_package(OpenCV 3.2.0 REQUIRED)
find_package(OpenCV 4.0 QUIET)
if(OpenCV_FOUND)
    message("Found OpenCV 4.0")
    set(BUILD_CAMERA_CALIBRATION 1)
else()
    message("Unable to find OpenCV 4.0")
    find_package(OpenCV 3.2.0 REQUIRED)
endif()
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

# Camera calibration
if (BUILD_CAMERA_CALIBRATION)
    message("Building camera calibration")
    add_executable(camera_calibration src/calibration/camera_calibration.cpp)
    target_link_libraries(camera_calibration ocam_camera ${OpenCV_LIBS})
    #add_executable(record_images src/calibration/record_images.cpp)
    #target_link_libraries(record_images ${OpenCV_LIBS} ocam_camera)
endif()

# AprilTags
add_library(detector
        src/detector/detector.cpp
        src/detector/transform.cpp
        src/detector/tag.cpp
        src/detector/transforms_cache.cpp)
target_link_libraries(detector ${apriltags_LIBRARIES})

# Tracker node
add_executable(tracker_node src/tracker_node.cpp)
target_link_libraries(tracker_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ocam_camera
        apriltags_wrapper
        camera_info)
