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
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;
  // Instantiate cameras
  /*tracker::Camera *camera1 = initializeOCam("/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0",
                                             width, height);*/

  // Start threads
  //Thread thread0(tracker::right_camera);
  Thread thread1(tracker::right_camera);
  ros::spin();

  // Wait and exit
  //thread0.join();
  thread1.join();
  return 0;
}
