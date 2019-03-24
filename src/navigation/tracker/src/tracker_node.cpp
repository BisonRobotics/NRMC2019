#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <tracker/detector/detector.h>
#include <tracker/camera/ocam_camera.h>
#include <tracker/thread/thread.h>

// Only load in main executable
#include <tracker/config/camera_config.h>
#include <tracker/config/tag_config.h>

using namespace tracker;
using std::string;
using ros::NodeHandle;


tracker::Camera* initializeOCam(CameraInfo info, uint brightness, uint exposure);
void trackerThread(string name, Camera *camera);

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;

  // Instantiate cameras
  tracker::Camera *camera0 = initializeOCam(tracker::right_camera, 19, 184);
  /*tracker::Camera *camera1 = initializeOCam("/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0",
                                             width, height);*/

  if (camera0 == nullptr)
  {
    ROS_WARN("Camera 0 handle invalid, shutting down");
    return 0;
  }

  // TODO initialize tag tfs (3rd argument true)
  Tag::init(10, ros::Duration(0.1), true);
  TagsVector tracker0_tags = Tag::getTags();
  Tag::setTransformCaches(&tracker0_tags, "right");

  // Start threads
  Thread thread0("tracker0", camera0, &tracker0_tags);
  //boost::thread thread1(trackerThread, "tracker1", camera1, &nh);
  ros::spin();

  // Wait and exit
  thread0.join();
  //thread1.join();
  return 0;
}

tracker::Camera* initializeOCam(CameraInfo info, uint brightness, uint exposure)
{
  tracker::Camera* camera = nullptr;
  while (camera == nullptr)
  {
    if (!ros::ok())
    {
      ROS_WARN("ROS is not ok");
      return nullptr;
    }
    try
    {
      // brightness (int)    : min=0 max=255 step=1 default=64 value=50
      // exposure_auto (menu)   : min=0 max=3 default=1 value=1
      // exposure_absolute (int)    : min=1 max=1000 step=1 default=55 value=90
      camera = new tracker::OCamCamera(info, 50, brightness, exposure);
    }
    catch (std::runtime_error &e)
    {
      ROS_WARN("%s", e.what());
      ros::Duration(2).sleep();
    }
  }
  return camera;
}
