#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>

#include <ocam/camera.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tracker/camera/ocam_camera.h>

#ifdef OPENCV_ENABLED
#include <opencv2/opencv.hpp>
#endif

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

int main (int argc, char* argv[])
{
  std::string dev_path = "/dev/video0";
  unsigned int  width = 1280, height = 720;

  // Initialize ROS
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  sensor_msgs::Image image_msg;
  image_msg.header.frame_id = "camera";
  image_msg.header.seq = 0;
  image_msg.header.stamp = ros::Time::now();
  image_msg.height = height;
  image_msg.width = width;
  image_msg.encoding = "mono8";
  image_msg.is_bigendian = 0;
  image_msg.step = width;
  image_msg.data.resize(width * height);

  // Initialize detector
  apriltag_family_t *tf = tag36h11_create();
  //apriltag_family_t *tf = tag25h10_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;      // Default = 2.0
  td->quad_sigma = 0.0;         // Default = 0.0
  td->refine_edges = 1;         // Default = 1
  td->decode_sharpening = 0.25; // Default = 0.25
  td->nthreads = 4;
  td->debug = 0;
  image_u8_t image =
  {
      .width = (int32_t)width,
      .height = (int32_t)height,
      .stride = (int32_t)width,
      .buf = image_msg.data.data()
  };

  // Initialize camera
  tracker::Camera *camera = nullptr;
  while (camera == nullptr)
  {
    try
    {
      camera = new tracker::OCamCamera(dev_path, width, height, 50, 50, 50);
    }
    catch (std::runtime_error &e)
    {
      ROS_WARN("%s", e.what());
      ros::Duration(5).sleep();
    }
  }
  camera->start();

#ifdef OPENCV_ENABLED
  cv::Mat cv_image(height, width, CV_8UC1, image.buf);
#endif

  // Main loop
  while (ros::ok())
  {
    // Get frame from camera (blocking)
    try
    {
      camera->getFrame(image.buf);
    }
    catch (std::runtime_error &e)
    {
      ROS_WARN("%s", e.what());
      camera->stop();
      camera->start();
      continue;
    }

    // Look for detector
    zarray_t *detections = apriltag_detector_detect(td, &image);
    //ROS_INFO("Tags detected: %i", zarray_size(detections));
    printf("[Detections] ");
    apriltag_pose_t pose;
    pose.t = matd_create(4, 4);
    pose.R = matd_create(4, 4);
    apriltag_detection_info_t info;
    info.tagsize = 0.2;
    info.fx = 1007.4436610527366;
    info.cx = 638.8631038728623;
    info.fy = 1004.6555107631117;
    info.cy = 351.5669704941244;
    apriltag_detection_t *det;
    for (int i = 0; i < zarray_size(detections); i++)
    {
      zarray_get(detections, i, &det);
      info.det = det;
      estimate_tag_pose(&info, &pose);
      printf("[%.2f, %.2f, %.2f], ", pose.t->data[0], pose.t->data[1], pose.t->data[2]);
    }
    printf("\n");

/*#ifdef OPENCV_ENABLED
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      line(cv_image, cv::Point(det->p[0][0], det->p[0][1]),
           cv::Point(det->p[1][0], det->p[1][1]),
           cv::Scalar(0xff), 2);
      line(cv_image, cv::Point(det->p[0][0], det->p[0][1]),
           cv::Point(det->p[3][0], det->p[3][1]),
           cv::Scalar(0xff), 2);
      line(cv_image, cv::Point(det->p[1][0], det->p[1][1]),
           cv::Point(det->p[2][0], det->p[2][1]),
           cv::Scalar(0xff), 2);
      line(cv_image, cv::Point(det->p[2][0], det->p[2][1]),
           cv::Point(det->p[3][0], det->p[3][1]),
           cv::Scalar(0xff), 2);

      std::stringstream ss;
      ss << det->id;
      cv::String text = ss.str();
      int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontscale = 1.0;
      int baseline;
      cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
                                  &baseline);
      putText(cv_image, text, cv::Point(det->c[0]-textsize.width/2,
                                 det->c[1]+textsize.height/2),
              fontface, fontscale, cv::Scalar(0xff), 2);
    }
#endif*/
    pub.publish(image_msg);
    apriltag_detections_destroy(detections);
  }

  // Exit
  //tag25h10_destroy(tf);
  tag36h11_destroy(tf);
  camera->stop();
  return 0;
}
