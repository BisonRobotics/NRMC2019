cmake_minimum_required(VERSION 2.8.3)
project(camera_calibration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(OpenCV 4.0 QUIET)
if(OpenCV_FOUND)
    message("Found OpenCV 4.0")

    catkin_package()

    include_directories(include)
    include_directories(${OpenCV_INCLUDE_DIRS})

    # Camera calibration
    message("Building camera calibration")
    add_executable(camera_calibration
      src/utility.cpp src/camera.cpp
      src/camera_calibration.cpp src/camera_info.cpp src/ocam_camera.cpp)
    target_link_libraries(camera_calibration pthread udev v4l2
      ${catkin_LIBRARIES} ${OpenCV_LIBS})

else()
    message("Unable to find OpenCV 4.0")
endif()
