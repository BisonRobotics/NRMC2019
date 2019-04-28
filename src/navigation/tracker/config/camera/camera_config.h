#ifndef TRACKER_CONFIG_H
#define TRACKER_CONFIG_H

#include <camera/camera_info.h>

namespace tracker
{

  double right_camera_data[9] =
      {971.2521738528383, 0,                 640,
       0,                 971.2521738528383, 360,
       0,                 0,                 1};
  double right_camera_distortion_data[5] =
      {-4.6063912654565564e-01, 2.8242818737094599e-01, 0, 0, -1.0420058130012258e-01};

  tracker::CameraInfo right_camera("right_camera",
    "/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0", 1280, 720,
    cv::Mat_<double>(3, 3, right_camera_data), cv::Mat_<double>(5,1, right_camera_distortion_data));

}

#endif //TRACKER_CONFIG_H
