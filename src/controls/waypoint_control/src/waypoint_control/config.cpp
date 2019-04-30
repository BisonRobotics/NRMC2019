#include <waypoint_control/config.h>

using namespace waypoint_control;

Config::Config(ros::NodeHandle *nh) : utilities::Config("waypoint_control")
{
  loadParam(nh, "rate",                 rate_,                  50.00);
  loadParam(nh, "max_acceleration",     max_acceleration_,       1.00);
  loadParam(nh, "max_duty",             max_duty_,               0.30);
  loadParam(nh, "max_in_place_duty",    max_in_place_duty_,      0.30);
  loadParam(nh, "min_in_place_duty",    min_in_place_duty_,      0.05);
  loadParam(nh, "max_driving_duty",     max_driving_duty_,       0.30);
  loadParam(nh, "min_driving_duty",     min_driving_duty_,       0.10);
  loadParam(nh, "max_manual_duty",      max_manual_duty_,        0.30);
  loadParam(nh, "min_manual_duty",      min_manual_duty_,        0.05);
  loadParam(nh, "in_place_k",           in_place_k_,             0.40);
  loadParam(nh, "driving_kx",           driving_kx_,             5.00);
  loadParam(nh, "driving_ky",           driving_ky_,             5.00);
  loadParam(nh, "full_autonomy",        full_autonomy_,         false);
  loadParam(nh, "voltage_compensation", voltage_compensation_,  false);
  loadParam(nh, "min_voltage",          min_voltage_,            30.0);
  loadParam(nh, "full_voltage",         full_voltage_,           42.0);
  loadParam(nh, "max_compensated_duty", max_compensated_duty_,   0.50);
  loadParam(nh, "battery_filter_k",     battery_filter_k_,       0.03);
  dt_ = 1.0/rate();
}

const double &Config::dt()
{
  return dt_;
}

const double &Config::rate()
{
  return rate_;
}

const double &Config::maxAcceleration()
{
  return max_acceleration_;
}

const double &Config::maxDuty()
{
  return max_duty_;
}

const double &Config::maxInPlaceDuty()
{
  return max_in_place_duty_;
}

const double &Config::minInPlaceDuty()
{
  return min_in_place_duty_;
}

const double &Config::maxDrivingDuty()
{
  return max_driving_duty_;
}

const double &Config::minDrivingDuty()
{
  return min_driving_duty_;
}

const double &Config::maxManualDuty()
{
  return max_manual_duty_;
}

const double &Config::minManualDuty()
{
  return min_manual_duty_;
}

const double &Config::inPlaceK()
{
  return in_place_k_;
}

const double &Config::drivingKx()
{
  return driving_kx_;
}

const double &Config::drivingKy()
{
  return driving_ky_;
}

const bool &Config::fullAutonomy()
{
  return full_autonomy_;
}

const bool &Config::voltageCompensation()
{
  return voltage_compensation_;
}

const double &Config::minVoltage()
{
  return min_voltage_;
}

const double &Config::fullVoltage()
{
  return full_voltage_;
}

const double &Config::maxCompensatedDuty()
{
  return max_compensated_duty_;
}

const double &Config::batteryFilterK()
{
  return battery_filter_k_;
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