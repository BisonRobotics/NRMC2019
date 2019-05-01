#include <waypoint_control/config.h>

using namespace waypoint_control;

Config::Config(ros::NodeHandle *nh) : utilities::Config("waypoint_control")
{
  // Load launch params
  loadParam(nh, "rate",                 rate_,                  50.00);
  loadParam(nh, "max_acceleration",     max_acceleration_,       1.00);
  loadParam(nh, "max_duty",             max_duty_,               0.30);
  loadParam(nh, "max_manual_duty",      max_manual_duty_,        0.30);
  loadParam(nh, "min_manual_duty",      min_manual_duty_,        0.05);
  loadParam(nh, "full_autonomy",        full_autonomy_,         false);
  loadParam(nh, "voltage_compensation", voltage_compensation_,  false);
  loadParam(nh, "min_voltage",          min_voltage_,            30.0);
  loadParam(nh, "full_voltage",         full_voltage_,           42.0);
  loadParam(nh, "max_compensated_duty", max_compensated_duty_,   0.50);
  loadParam(nh, "battery_filter_k",     battery_filter_k_,       0.03);
  dt_ = 1.0/rate();

  // Load yaml params
  if (nh->hasParam("drive_profiles"))
  {
    ROS_INFO("Found drive_profiles parameter, attempting to load profiles");
    XmlRpc::XmlRpcValue xml_drive_profiles;
    nh->getParam("drive_profiles", xml_drive_profiles);
    ROS_ASSERT(xml_drive_profiles.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < xml_drive_profiles.size(); i++)
    {
      drive_profiles_.emplace_back(xml_drive_profiles[i]["max_in_place_duty"], xml_drive_profiles[i]["min_in_place_duty"],
                                   xml_drive_profiles[i]["max_driving_duty"],  xml_drive_profiles[i]["min_driving_duty"],
                                   xml_drive_profiles[i]["in_place_k"],        xml_drive_profiles[i]["driving_kx"],
                                   xml_drive_profiles[i]["driving_ky"]);
    }
  }
  else
  {
    ROS_ERROR("Unable to find drive_profiles");
    throw std::runtime_error("drive_profiles param not found");
  }
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

const double &Config::maxManualDuty()
{
  return max_manual_duty_;
}

const double &Config::minManualDuty()
{
  return min_manual_duty_;
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

const DriveProfile &Config::driveProfile(int i)
{
  return drive_profiles_[i];
}

DriveProfile::DriveProfile(double max_in_place_duty, double min_in_place_duty, double max_driving_duty,
                           double min_driving_duty, double in_place_k, double driving_kx, double driving_ky) :
                           max_in_place_duty_(max_in_place_duty), min_in_place_duty_(min_in_place_duty),
                           max_driving_duty_(max_driving_duty), min_driving_duty_(min_driving_duty),
                           in_place_k_(in_place_k), driving_kx_(driving_kx), driving_ky_(driving_ky)
{
  ROS_INFO("[DriveProfile::DriveProfile]: %f, %f, %f, %f, %f, %f, %f", max_in_place_duty_, min_in_place_duty_,
      max_driving_duty_, min_driving_duty_, in_place_k_, driving_kx_, driving_ky_);
}


const double &DriveProfile::maxInPlaceDuty() const
{
  return max_in_place_duty_;
}

const double &DriveProfile::minInPlaceDuty() const
{
  return min_in_place_duty_;
}

const double &DriveProfile::maxDrivingDuty() const
{
  return max_driving_duty_;
}

const double &DriveProfile::minDrivingDuty() const
{
  return min_driving_duty_;
}

const double &DriveProfile::inPlaceK() const
{
  return in_place_k_;
}

const double &DriveProfile::drivingKx() const
{
  return driving_kx_;
}

const double &DriveProfile::drivingKy() const
{
  return driving_ky_;
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
