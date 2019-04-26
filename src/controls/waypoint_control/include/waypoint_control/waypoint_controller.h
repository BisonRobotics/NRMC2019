#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>

#include <utilities/filter.h>
#include <waypoint_control/waypoint_control_config.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/Geometry>
#include <waypoint_control/feedback.h>
#include <waypoint_control/Debug.h>


namespace waypoint_control
{
  using Rotation2D = Eigen::Rotation2D<double>;

  double clampAcceleration(double value, double last_value, double limit, double dt);

  class WaypointController
  {
  public:
    WaypointController(iVescAccess *front_left,   iVescAccess *front_right,
                       iVescAccess *back_right, iVescAccess *back_left, Config *config);

    void update(bool manual_safety, bool autonomy_safety,
                tf2::Transform transform, double left, double right);
    void updateControls(const tf2::Transform &transform);
    void setPoint(double left, double right, bool reverse = false);
    void setControlState(ControlState goal);
    void setControlState(ControlState goal, const Waypoints &waypoint);
    size_t remainingWaypoints();
    void stop();
    ControlState getControlState() const;
    Debug getDebugInfo() const;

  private:
    double last_left, last_right, dt;
    Config *config;
    Debug debug_info;
    iVescAccess *fl, *fr, *br, *bl;
    ControlState state;
    WaypointState waypoint_state;
    Waypoints waypoints;
    Feedback feedback, last_feedback;
    tf2::Transform last_transform;

  };

}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
