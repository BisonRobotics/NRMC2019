#include <waypoint_control/waypoint_control_config.h>

using namespace waypoint_control;

Config::Config(ros::NodeHandle *nh) : utilities::Config("waypoint_control")
{
  loadParam(nh, "rate",              rate,              50.00);
  loadParam(nh, "max_acceleration",  max_acceleration,   0.01);
  loadParam(nh, "max_duty",          max_duty,           0.30);
  loadParam(nh, "max_in_place_duty", max_in_place_duty,  0.20);
  loadParam(nh, "min_in_place_duty", min_in_place_duty,  0.10);
  loadParam(nh, "max_driving_duty",  max_driving_duty,   0.40);
  loadParam(nh, "min_driving_duty",  min_driving_duty,   0.20);
  loadParam(nh, "max_manual_duty",   max_manual_duty,    0.20);
  loadParam(nh, "min_manual_duty",   min_manual_duty,    0.05);
  loadParam(nh, "in_place_k",        in_place_k,         1.00);
  loadParam(nh, "driving_kx",        driving_kx,         1.00);
  loadParam(nh, "driving_ky",        driving_ky,         1.00);
}

std::string waypoint_control::to_string(ControlState state)
{
  switch (state)
  {
    case ControlState::error:
      return "error";
    case ControlState::ready:
      return "ready";
    case ControlState::new_goal:
      return "new_goal";
    case ControlState::in_progress:
      return "in_progress";
    case ControlState::finished:
      return "finish";
    case ControlState::cancel:
      return "cancel";
    case ControlState::manual:
      return "manual";
  }
}

std::string waypoint_control::to_string(WaypointState state)
{
  switch (state)
  {
    case WaypointState::error:
      return "error";
    case WaypointState::ready:
      return "ready";
    case WaypointState::initial_angle_correction:
      return "initial_angle_correction";
    case WaypointState::driving:
      return "driving";
    case WaypointState::angle_correction:
      return "angle_correction";
    case WaypointState::final_angle_correction:
      return "final_angle_correction";
    case WaypointState::finished:
      return "finish";
  }
}

WaypointControlResult waypoint_control::toResult(ControlState state)
{
  WaypointControlResult result;
  result.control_state = (WaypointControlResult::_control_state_type)state;
  return result;
}

ControlState waypoint_control::toControlState(WaypointControlGoal goal)
{
  return (ControlState)goal.control_state;
}