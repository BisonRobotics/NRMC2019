#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>

#include <ocam/camera.hpp>
#include <tracker/apriltags/apriltags.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tracker/camera/ocam_camera.h>
#include <tracker/config/config.h>

#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

extern "C"
{
#include "apriltags/apriltag.h"
#include "apriltags/tags/tag16h6.h"
#include "apriltags/tags/tag25h10.h"
#include "apriltags/tags/tag36h11.h"
#include "apriltags/common/getopt.h"
#include "apriltags/apriltag_pose.h"
}

// TODO retry tag16h6 with a check for tag size
// TODO generate tag36hX with a Hamming distance X
//  greater than 11 (less tags, better accuracy)
// TODO calibrate camera

using namespace tracker;
using std::string;
using ros::NodeHandle;

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


void trackerThread(string camera_name, Camera *camera, NodeHandle *nh)
{
  // Initialize ROS image stuff
  image_transport::ImageTransport it(*nh);
  image_transport::Publisher pub = it.advertise(camera_name + "/image", 1);
  //const int drops = 4; // Publish one of every ten images
  const int drops = 0;
  int drop_count = 0;

  sensor_msgs::Image image_msg;
  image_msg.header.frame_id = camera_name;
  image_msg.header.seq = 0;
  image_msg.header.stamp = ros::Time::now();
  image_msg.height = camera->getHeight();
  image_msg.width = camera->getWidth();
  image_msg.encoding = "mono8";
  image_msg.is_bigendian = 0;
  image_msg.step = camera->getWidth();
  image_msg.data.resize(camera->getWidth() * camera->getHeight());

  // Initialize apriltags
  /*apriltag_family_t *tf = tag36h11_create();
  //apriltag_family_t *tf = tag25h10_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 4.0;      // Default = 2.0
  td->quad_sigma = 0.0;         // Default = 0.0
  td->refine_edges = 1;         // Default = 1
  td->decode_sharpening = 0.25; // Default = 0.25
  td->nthreads = 4;
  td->debug = 0;
  image_u8_t image =
  {
    .width = (int32_t)camera->getWidth(),
    .height = (int32_t)camera->getHeight(),
    .stride = (int32_t)camera->getWidth(),
    .buf = image_msg.data.data()
  };*/

  AprilTagDetector detector(camera->getInfo(), image_msg.data.data());


  // Start camera
  camera->start();

  // Main loop
  while (ros::ok())
  {
    // Get frame from camera (blocking)
    bool success = false;
    while (!success)
    {
      try
      {
        camera->getFrame(detector.getBuffer());
        detector.detect();
      }
      catch (std::runtime_error &e)
      {
        ROS_WARN("%s", e.what());
        try
        {
          camera->stop();
          camera->start();
        }
        catch (std::runtime_error &e)
        {
          ROS_WARN("%s", e.what());
        }
        continue;
      }
      success = true;
    }

    // Occasionally publish an image
    if (drop_count++ >= drops)
    {
      pub.publish(image_msg);
      drop_count = 0;
    }
  }

  // Exit
  //tag25h10_destroy(tf);
  camera->stop();
}

int main (int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "apriltag_tracker");
  ros::NodeHandle nh;

  // Instantiate cameras
  tracker::Camera *camera0 = initializeOCam(tracker::right_camera, 64, 200);
  /*tracker::Camera *camera1 = initializeOCam("/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0",
                                             width, height);*/

  if (camera0 == nullptr)
  {
    ROS_WARN("Camera 0 handle invalid, shutting down");
    return 0;
  }

  // Start threads
  boost::thread thread0(trackerThread, "tracker0", camera0, &nh);
  //boost::thread thread1(trackerThread, "tracker1", camera1, &nh);
  ros::spin();

  // Wait and exit
  thread0.join();
  //thread1.join();
  return 0;
}
