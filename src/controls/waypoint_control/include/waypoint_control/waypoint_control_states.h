#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H

#include <waypoint_control/WaypointControlAction.h>
#include <string>

namespace waypoint_control
{
  typedef std::vector<Waypoint> Waypoints;

  enum class ControlState
  {
    error = 0,
    ready,
    new_goal,
    in_progress,
    cancel,
    manual
  };

  enum class WaypointState
  {
    error = 0,
    ready,
    starting_orientation,
    initial_angle_correction,
    driving,
    angle_correction,
    final_angle_correction,
  };

  WaypointControlResult toResult(ControlState state);
  ControlState toControlState(WaypointControlGoal goal);
  std::string to_string(ControlState state);
}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
