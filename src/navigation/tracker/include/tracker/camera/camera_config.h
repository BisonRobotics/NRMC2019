#ifndef TRACKER_CAMERA_CONFIG_H
#define TRACKER_CAMERA_CONFIG_H

#include <tracker/camera/camera_info.h>

namespace tracker
{

  double right_camera_data[9] =
      {968.4808579735501, 0, 640,
       0, 968.4808579735501, 360,
       0,                 0,   1};
  double right_camera_distortion_data[5] =
      {-4.3927531331725828e-01, 2.6317484443226025e-01, 0.0, 0.0,-9.9331952846763508e-02};

  tracker::CameraInfo right_camera("right",
    "/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183225-video-index0", 1280, 720,
    cv::Mat_<double>(3, 3, right_camera_data), cv::Mat_<double>(5,1, right_camera_distortion_data));

  double left_camera_data[9] =
      {882.0010807541458, 0, 640,
       0, 882.0010807541458, 360,
       0,                 0,   1};
  double left_camera_distortion_data[5] =
      {-3.6820179831442307e-01, 1.8397750207877953e-01, 0.0, 0.0, -5.7895335957268895e-02};

  tracker::CameraInfo left_camera("left",
                                   "/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1MGN-U_SN_2C183178-video-index0", 1280, 720,
                                   cv::Mat_<double>(3, 3, right_camera_data), cv::Mat_<double>(5,1, right_camera_distortion_data));

}

#endif //TRACKER_CAMERA_CONFIG_H
