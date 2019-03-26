#ifndef TRACKER_UTIL_H
#define TRACKER_UTIL_H

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

namespace tracker
{
  void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw);
  double getTheta(tf2::Quaternion orientation);
}

#endif //TRACKER_UTIL_H
