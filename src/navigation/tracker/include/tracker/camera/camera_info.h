#ifndef TRACKER_CAMERA_PARAMETERS_H
#define TRACKER_CAMERA_PARAMETERS_H

#include <stdint.h>
#include <sys/types.h>

namespace tracker
{
  class CameraInfo
  {
  public:
    CameraInfo(uint width, uint height, double fx, double fy, double cx, double cy);
    const uint width, height;
    const double fx, fy, cx, cy;
  };
}

#endif //TRACKER_CAMERA_PARAMETERS_H
