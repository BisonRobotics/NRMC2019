#ifndef WAYPOINT_CONTROL_FEEDBACK_H
#define WAYPOINT_CONTROL_FEEDBACK_H

#include <waypoint_control/config.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/Geometry>

namespace waypoint_control
{
  using Rotation2D = Eigen::Rotation2D<double>;

  class Feedback
  {
  public:
    Feedback();
    Feedback(const geometry_msgs::Pose2D &pose, const Waypoint &W);

    double x();
    double y();
    double r();
    double theta();

  private:
    double x_;
    double y_;
    double r_;
    double theta_;
  };
}

#endif //WAYPOINT_CONTROL_FEEDBACK_H
