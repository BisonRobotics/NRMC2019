#include <tracker/util.h>

void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 matrix;
  matrix.setRotation(q);
  matrix.getRPY(roll, pitch, yaw);
}

double getTheta(tf2::Quaternion orientation)
{
  double roll, pitch, yaw;
  getRPY(orientation, roll, pitch, yaw);
  return pitch;
}