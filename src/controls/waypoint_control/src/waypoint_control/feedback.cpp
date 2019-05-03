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

Feedback::Feedback(const geometry_msgs::Pose2D &P, const Waypoint &W)
{
  // Get feedback
  if (W.drive_profile == 2)
  {
    double dx = W.pose.x - P.x;
    if (W.reverse)
    {
      theta_ = Rotation2D(-P.theta).smallestAngle();
    }
    else
    {
      theta_ = (Rotation2D(pi) * Rotation2D(-P.theta)).smallestAngle();
    }
    r_ = std::abs(dx);
    x_ = (dx >= 0.0 ? 1.0 : -1.0) * r_ * cos(theta_);
    y_ = r_ * sin(theta_);
  }
  else
  {
    double dy = W.pose.y - P.y;
    double dx = W.pose.x - P.x;
    r_ = sqrt(dx*dx + dy*dy);

    if (W.reverse)
    {
      theta_ = (Rotation2D(pi) * Rotation2D(std::atan2(dy, dx)) * Rotation2D(-P.theta)).smallestAngle();
    }
    else
    {
      theta_ = (Rotation2D(std::atan2(dy, dx)) * Rotation2D(-P.theta)).smallestAngle();
    }

    x_ = r_ * cos(theta_);
    y_ = r_ * sin(theta_);
  }
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
