#include <stdio.h>
#include <errno.h>
#include <iostream>
#include <fstream>

#include <ocam/camera.hpp>

#include <camera/ocam_camera.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread.hpp>

// TODO retry tag16h6 with a check for tag size
// TODO generate tag36hX with a Hamming distance X
//  greater than 11 (less tags, better accuracy)
// TODO calibrate camera

using namespace tracker;
using std::string;

tracker::Camera* initializeOCam(std::string device_path, uint width, uint height)
{
  CameraInfo info(width, height, 1007.4436610527366, 638.8631038728623, 1004.6555107631117, 351.5669704941244);

  tracker::Camera* camera = nullptr;
  while (camera == nullptr)
  {
    try
    {
      // brightness (int)    : min=0 max=255 step=1 default=64 value=50
      // exposure_auto (menu)   : min=0 max=3 default=1 value=1
      // exposure_absolute (int)    : min=1 max=1000 step=1 default=55 value=90
      camera = new tracker::OCamCamera(info, device_path, 50, 64, 80);
    }
    catch (std::runtime_error &e)
    {
      printf("%s", e.what());
      usleep(1000000);
    }
  }
  return camera;
}

int main (int argc, char* argv[])
{
  // Instantiate cameras
  unsigned int  width = 1280, height = 720;
  tracker::Camera *camera = initializeOCam("/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0",
                                             width, height);
  /*tracker::Camera *camera1 = initializeOCam("/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0",
                                             width, height);*/

  if (camera == nullptr)
  {
    printf("Camera 0 was not able to be opened\n");
    return 0;
  }

  cv::Mat frame(height, width, CV_8UC1);
  cv::imshow("frame", frame);
  camera->start();
  int captures = 0;

  while (1)
  {
    try
    {
      camera->getFrame(frame.data);
      cv::rotate(frame, frame, cv::ROTATE_180);
      cv::imshow("frame", frame);
      int key = cv::waitKey(1);
      if (key == 'q')
      {
        break;
      }
      else if (key == 'c')
      {
        cv::imwrite("/tmp/image_captures/frame" + std::to_string(captures) + ".png", frame);
        printf("Wrote /tmp/image_captures/capture%i\n", captures++);
      }
    }
    catch (std::runtime_error &e)
    {
      printf("%s\n", e.what());
      usleep(100000);
    }
  }

  printf("Finished\n");


  return 0;
}
