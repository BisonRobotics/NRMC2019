#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>

#include <utilities/filter.h>
#include <waypoint_control/waypoint_control_states.h>
#include <tf2/LinearMath/Transform.h>

namespace waypoint_control
{
  class WaypointController
  {
  public:
    WaypointController(iVescAccess *front_left,   iVescAccess *front_right,
                       iVescAccess *back_right, iVescAccess *back_left, double max_velocity);

    void update(bool manual_safety, bool autonomy_safety,
                tf2::Transform transform, double left, double right);
    void setPoint(double left, double right);
    void setControlState(ControlState goal);
    void setControlState(ControlState goal, const Waypoints &waypoint);
    void stop();
    ControlState getControlState() const;

  private:
    double max_velocity;
    iVescAccess *fl, *fr, *br, *bl;
    ControlState state;
    WaypointState waypoint_state;
    Waypoints waypoints;
    tf2::Transform last_transform;
  };

}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
