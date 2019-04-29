#include <tracker/config.h>

using namespace tracker;

Config::Config(ros::NodeHandle *base_nh, ros::NodeHandle *nh, std::string name) :
  utilities::Config("tracker"), name(name)
{
  loadParam(base_nh, "max_initialization_velocity", max_initialization_velocity, 0.3);
  loadParam(base_nh, "max_scan_velocity", max_scan_velocity, 0.1);
  loadParam(base_nh, "max_velocity", max_velocity, 0.2);
  loadParam(base_nh, "k", k, 4.0);
  loadParam(base_nh, "brightness", brightness, 54);
  loadParam(base_nh, "exposure", exposure, 89);
  loadParam(nh, "stepper_controller_id", stepper_controller_id, 1);
  loadParam(nh, "stepper_client_id", stepper_client_id, 3);
}