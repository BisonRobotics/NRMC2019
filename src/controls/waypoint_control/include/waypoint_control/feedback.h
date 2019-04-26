#ifndef WAYPOINT_CONTROL_FEEDBACK_H
#define WAYPOINT_CONTROL_FEEDBACK_H

#include <waypoint_control/waypoint_control_config.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/Geometry>

namespace waypoint_control
{
  using Rotation2D = Eigen::Rotation2D<double>;

  class Feedback
  {
  public:
    Feedback();
    Feedback(tf2::Transform robot, Waypoint W);

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
