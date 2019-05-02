#include <dig_control/config.h>

using namespace dig_control;

Config::Config(ros::NodeHandle *nh) : utilities::Config("dig_control")
{
  loadParam(nh, "debug",                               debug_,                               true);
  loadParam(nh, "floor_test",                          floor_test_,                          true);
  loadParam(nh, "full_autonomy",                       full_autonomy_,                      false);
  loadParam(nh, "voltage_compensation",                voltage_compensation_,               false);
  loadParam(nh, "min_voltage",                         min_voltage_,                         30.0);
  loadParam(nh, "full_voltage",                        full_voltage_,                        42.0);
  loadParam(nh, "max_compensated_duty",                max_compensated_duty_,                0.50);
  loadParam(nh, "battery_filter_k",                    battery_filter_k_,                    0.03);
  loadParam(nh, "central_drive_current_filter_k",      central_drive_current_filter_k_,      0.04);
  loadParam(nh, "central_drive_dig_current_threshold", central_drive_dig_current_threshold_, 8.00);
  loadParam(nh, "battery_filter_k",                    battery_filter_k_,                    0.03);
  loadParam(nh, "central_drive_angle_filter_k",        central_drive_angle_filter_k_,        0.10);
  loadParam(nh, "current_filter_k",                    current_filter_k_,                    0.04);
  loadParam(nh, "bucket_filter_k",                     bucket_filter_k_,                     0.03);
}

const CentralDriveAngles &Config::centralDriveAngles()
{
  return central_drive_angles_;
}

const CentralDriveDuty &Config::centralDriveDuty()
{
  return central_drive_duty_;
}

const BackhoeDuty &Config::backhoeDuty()
{
  return backhoe_duty_;
}

const VibratorDuty &Config::vibratorDuty()
{
  return vibrator_duty_;
}

const BucketDuty &Config::bucketDuty()
{
  return bucket_duty_;
}

const bool &Config::debug()
{
  return debug_;
}

const bool &Config::floorTest()
{
  return floor_test_;
}

const bool &Config::voltageCompensation()
{
  return voltage_compensation_;
}

const double &Config::minVoltage()
{
  return min_voltage_;
}

const double &Config::startVoltage()
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

const double &Config::centralDriveAngleFilterK()
{
  return central_drive_angle_filter_k_;
}

const double &Config::centralDriveCurrentFilterK()
{
  return central_drive_current_filter_k_;
}

const double &Config::centralDriveDigCurrentThreshold()
{
  return central_drive_dig_current_threshold_;
}

const double &Config::currentFilterK()
{
  return current_filter_k_;
}

const double &Config::bucketFilterK()
{
  return bucket_filter_k_;
}

const bool &Config::fullAutonomy()
{
  return full_autonomy_;
}

std::string dig_control::to_string(ControlState state)
{
  switch (state)
  {
    case ControlState::initialize:
      return "initialize";
    case ControlState::dig:
      return "dig";
    case ControlState::manual:
      return "manual";
    case ControlState::error:
      return "error";
    case ControlState::ready:
      return "ready";
    case ControlState::finish_dump:
      return "finish_dump";
    case ControlState::finish_dig:
      return "finish_dig";
    case ControlState::dump:
      return "dump";
  }
  return "Not defined";
}

std::string dig_control::to_string(DigState state)
{
  switch (state)
  {
    case DigState::initialize:
      return "initialize";
    case DigState::dig_transition:
      return "dig_transition";
    case DigState::digging:
      return "digging";
    case DigState::closing_backhoe:
      return "closing_backhoe";
    case DigState::dump_transition:
      return "dump_transition";
    case DigState::moving_flaps_up:
      return "moving_flaps_up";
    case DigState::stow:
      return "stow";
  }
  return "Not defined";
}

std::string dig_control::to_string(CentralDriveState state)
{
  switch (state)
  {
    case CentralDriveState::at_bottom_limit:
      return "at_bottom_limit";
    case CentralDriveState::digging:
      return "digging";
    case CentralDriveState::near_digging:
      return "near_digging";
    case CentralDriveState::flap_transition_down:
      return "flap_transition_down";
    case CentralDriveState::near_dump_point:
      return "near_dump_point";
    case CentralDriveState::at_dump_point:
      return "at_dump_point";
    case CentralDriveState::flap_transition_up:
      return "flap_transition_up";
    case CentralDriveState::at_top_limit:
      return "at_top_limit";
  }
  return "Not defined";
}

std::string dig_control::to_string(BackhoeState state)
{
  switch (state)
  {
    case BackhoeState::open:
      return "open";
    case BackhoeState::closed:
      return "closed";
    case BackhoeState::traveling:
      return "traveling";
    case BackhoeState::stuck:
      return "stuck";
  }
  return "Not defined";
}

std::string dig_control::to_string(BucketState state)
{
  switch (state)
  {
    case BucketState::down:
      return "down";
    case BucketState::stuck:
      return "stuck";
    case BucketState::traveling:
      return "traveling";
    case BucketState::up:
      return "up";
  }
  return "Not defined";
}
