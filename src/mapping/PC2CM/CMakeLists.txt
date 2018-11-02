cmake_minimum_required(VERSION 2.8.3)
project(pc2cm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	nav_msgs
	navigation_msgs
	sensor_msgs
	pcl_ros
)

find_package(OpenCV REQUIRED)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)


catkin_package(
	INCLUDE_DIRS include/
	LIBRARIES pc2_processor
    CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
    DEPENDS OpenCV
)

include_directories (
    include/
	include/pc2_processor/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}

)

add_library(pc2_processor src/pc2_processor/pc2_processor.cpp)

add_executable(pc2cm_master src/pc2cm_master.cpp)
target_link_libraries(pc2cm_master pc2_processor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

catkin_add_gtest(pc2_processor_tests include/pc2_processor/pc2_processor.h src/pc2_processor/test/pc2_processor_tests.cpp)
if(TARGET pc2_processor_tests)
target_link_libraries(pc2_processor_tests
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    gmock_main
    pc2_processor
)
endif()