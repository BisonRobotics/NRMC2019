#include <waypoint_control/feedback.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

using std::sqrt;
using std::cos;
using std::sin;
using std::acos;
using std::asin;
using std::atan2;

using namespace waypoint_control;

Feedback::Feedback() : x_(0.0), y_(0.0), r_(0.0), theta_(0.0)
{}

Feedback::Feedback(tf2::Transform R, Waypoint W)
{
  // Make sure that the robot rotation is valid
  auto Q1 = R.getRotation();
  double Q1m = Q1.x()*Q1.x() + Q1.y()*Q1.y() + Q1.z()*Q1.z() + Q1.w()*Q1.w();
  if (std::abs(Q1m - 1.0) > 1.0e-6)
  {
    throw std::runtime_error("[Feedback::Feedback]: Robot orientation isn't normalized");
  }

  // Make sure that the waypoint rotation is valid, correct if not
  auto Q2 = W.pose.orientation;
  double Q2m = Q2.x*Q2.x + Q2.y*Q2.y + Q2.z*Q2.z + Q2.w*Q2.w;
  if (std::abs(Q2m - 1.0) > 1.0e-6)
  {
    ROS_WARN("[Feedback::Feedback]: Waypoint quaternion isn't normalized, assuming default orientation");
    W.pose.orientation.x = 0.0;
    W.pose.orientation.y = 0.0;
    W.pose.orientation.z = 0.0;
    W.pose.orientation.w = 1.0;
  }

  // Get feedback
  double dy = W.pose.position.y - R.getOrigin().y();
  double dx = W.pose.position.x - R.getOrigin().x();
  r_ = sqrt(dx*dx + dy*dy);

  double roll, pitch, yaw;
  tf2::Matrix3x3(R.getRotation()).getRPY(roll, pitch, yaw);
  if (W.reverse)
  {
    theta_ = (Rotation2D(pi) * Rotation2D(std::atan2(dy, dx)) * Rotation2D(-yaw)).smallestAngle();
  }
  else
  {
    theta_ = (Rotation2D(std::atan2(dy, dx)) * Rotation2D(-yaw)).smallestAngle();
  }

  x_ = r_ * cos(theta_);
  y_ = r_ * sin(theta_);
}

double Feedback::x()
{
  return x_;
}

double Feedback::y()
{
  return y_;
}

double Feedback::r()
{
  return r_;
}

double Feedback::theta()
{
  return theta_;
}

