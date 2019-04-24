#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>

#include <utilities/filter.h>
#include <waypoint_control/waypoint_control_states.h>
#include <waypoint_control/waypoint_config.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/Geometry>


namespace waypoint_control
{
  using Rotation2D = Eigen::Rotation2D<double>;

  Rotation2D getAngularError(const tf2::Transform &transform, const Waypoint &waypoint);

  class WaypointController
  {
  public:
    WaypointController(iVescAccess *front_left,   iVescAccess *front_right,
                       iVescAccess *back_right, iVescAccess *back_left, double max_velocity);

    void update(bool manual_safety, bool autonomy_safety,
                tf2::Transform transform, double left, double right);
    void updateControls(const tf2::Transform &transform);
    void setPoint(double left, double right);
    void setControlState(ControlState goal);
    void setControlState(ControlState goal, const Waypoints &waypoint);
    void stop();
    ControlState getControlState() const;

  private:
    Config config;
    double max_velocity;
    iVescAccess *fl, *fr, *br, *bl;
    ControlState state;
    WaypointState waypoint_state;
    Waypoints waypoints;
    Rotation2D angular_error;
    tf2::Transform last_transform;
  };

}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
