#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <tracker/detector.h>
#include <tracker/camera/ocam_camera.h>
#include <tracker/thread.h>

#include <stepper/stepper.h>
#include <tracker/camera/camera_config.h>

// Only load in main executable
//#include <tracker/config/camera_config.h>
//#include <tracker/config/tag_config.h>

using namespace tracker;
using std::string;
using ros::NodeHandle;


tracker::Camera* initializeOCam(CameraInfo info, uint brightness, uint exposure);

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "tracker");
  ros::NodeHandle base_nh("~");
  ros::NodeHandle left_nh("~/left");
  ros::NodeHandle right_nh("~/right");
  Config left_config(&base_nh, &left_nh, "left");
  Config right_config(&base_nh, &right_nh, "right");

  // Start threads
  Thread thread0(base_nh, left_nh, left_config, tracker::left_camera);
  Thread thread1(base_nh, right_nh, right_config, tracker::right_camera);
  ros::spin();

  // Wait and exit
  thread0.join();
  thread1.join();
  return 0;
}
