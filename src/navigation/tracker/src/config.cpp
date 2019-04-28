#include <tracker/config.h>

using namespace tracker;

Config::Config(ros::NodeHandle *base_nh, ros::NodeHandle *nh, std::string name) : name(name)
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

void Config::loadParam(ros::NodeHandle *nh, std::string name, std::string &param, std::string default_param)
{
  nh->param<std::string>(name, param, default_param);
  ROS_INFO("[tracker::Config::loadParam::%s]: %s = %s ", nh->getNamespace().c_str(), name.c_str(), param.c_str());
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, int &param, int default_param)
{
  nh->param<int>(name, param, default_param);
  ROS_INFO("[tracker::Config::loadParam]: %s = %i ", name.c_str(), param);
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, double &param, double default_param)
{
  nh->param<double>(name, param, default_param);
  ROS_INFO("[tracker::Config::loadParam]: %s = %f ", name.c_str(), param);
}