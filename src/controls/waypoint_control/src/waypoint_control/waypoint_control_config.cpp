#include <waypoint_control/waypoint_control_config.h>

using namespace waypoint_control;

Config::Config() :
    max_duty(default_config::MAX_DUTY),
    max_in_place_duty(default_config::MAX_IN_PLACE_DUTY),
    min_in_place_duty(default_config::MIN_IN_PLACE_DUTY),
    max_driving_duty(default_config::MAX_DRIVING_DUTY),
    min_driving_duty(default_config::MIN_DRIVING_DUTY),
    max_manual_duty(default_config::MAX_MANUAL_DUTY),
    in_place_k(default_config::IN_PLACE_K),
    driving_k(default_config::DRIVING_K)
{}

Config::Config(ros::NodeHandle *nh)
{
  nh->param<double>("max_duty", max_duty, default_config::MAX_DUTY);
  nh->param<double>("max_in_place_duty", max_in_place_duty, default_config::MAX_IN_PLACE_DUTY);
  nh->param<double>("min_in_place_duty", min_in_place_duty, default_config::MIN_IN_PLACE_DUTY);
  nh->param<double>("max_driving_duty", max_driving_duty, default_config::MAX_DRIVING_DUTY);
  nh->param<double>("min_driving_duty", min_driving_duty, default_config::MIN_DRIVING_DUTY);
  nh->param<double>("max_manual_duty", max_manual_duty, default_config::MAX_MANUAL_DUTY);
  nh->param<double>("min_manual_duty", min_manual_duty, default_config::MIN_MANUAL_DUTY);
  nh->param<double>("in_place_k", in_place_k, default_config::IN_PLACE_K);
  nh->param<double>("driving_k", driving_k, default_config::DRIVING_K);
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