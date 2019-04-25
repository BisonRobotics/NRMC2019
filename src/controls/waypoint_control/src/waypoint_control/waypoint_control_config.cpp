#include <waypoint_control/waypoint_control_config.h>

using namespace waypoint_control;

Config::Config() :
    initial_angular_variation(default_config::INITIAL_ANGULAR_VELOCITY),
    max_velocity(default_config::MAX_VELOCITY),
    max_linear_velocity(default_config::MAX_LINEAR_VELOCITY),
    max_angular_velocity(default_config::MAX_ANGULAR_VELOCITY) {}

Config::Config(ros::NodeHandle *nh)
{
  nh->param<double>("initial_angular_velocity", initial_angular_variation, default_config::INITIAL_ANGULAR_VELOCITY);
  nh->param<double>("max_velocity", max_velocity, default_config::MAX_VELOCITY);
  nh->param<double>("max_linear_velocity", max_linear_velocity, default_config::MAX_LINEAR_VELOCITY);
  nh->param<double>("max_angular_velocity", max_angular_velocity, default_config::MAX_ANGULAR_VELOCITY);
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
    case WaypointState::starting_orientation:
      return "starting_orientation";
    case WaypointState::initial_angle_correction:
      return "initial_angle_correction";
    case WaypointState::driving:
      return "driving";
    case WaypointState::angle_correction:
      return "angle_correction";
    case WaypointState::final_angle_correction:
      return "final_angle_correction";
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