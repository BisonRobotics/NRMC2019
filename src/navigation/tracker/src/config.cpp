#include <tracker/config.h>

using namespace tracker;

Config::Config(ros::NodeHandle *base_nh, ros::NodeHandle *nh, std::string name) :
  utilities::Config("tracker"), name_(name)
{
  loadParam(base_nh, "tag_switch_x",                tag_switch_x_,                2.50);
  loadParam(base_nh, "tag_switch_y",                tag_switch_y_,                1.75);
  loadParam(base_nh, "max_initialization_velocity", max_initialization_velocity_, 0.30);
  loadParam(base_nh, "initial_scan_velocity",       initial_scan_velocity_,       0.05);
  loadParam(base_nh, "max_scan_velocity",           max_scan_velocity_,           0.10);
  loadParam(base_nh, "max_velocity",                max_velocity_,                0.20);
  loadParam(base_nh, "k",                           k_,                           4.00);
  loadParam(base_nh, "brightness",                  brightness_,                   184);
  loadParam(base_nh, "exposure",                    exposure_,                      89);
  loadParam(nh,      "stepper_controller_id",       stepper_controller_id_,          1);
  loadParam(nh,      "stepper_client_id",           stepper_client_id_,              3);
}

const std::string &Config::name()
{
  return name_;
}


const double &Config::tagSwitchX()
{
  return tag_switch_x_;
}

const double &Config::tagSwitchY()
{
  return tag_switch_y_;
}

const double &Config::maxInitializationVelocity()
{
  return max_initialization_velocity_;
}

const double &Config::initialScanVelocity()
{
  return initial_scan_velocity_;
}


const double &Config::maxScanVelocity()
{
  return max_scan_velocity_;
}

const double &Config::maxVelocity()
{
  return max_velocity_;
}

const double &Config::k()
{
  return k_;
}

const int &Config::stepperControllerID()
{
  return stepper_controller_id_;
}

const int &Config::stepperClientID()
{
  return stepper_client_id_;
}

const int &Config::brightness()
{
  return brightness_;
}

const int &Config::exposure()
{
  return exposure_;
}