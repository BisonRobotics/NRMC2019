#include <utilities/config.h>

using namespace utilities;

Config::Config(std::string package_name) : package_name(package_name)
{

}


void Config::loadParam(ros::NodeHandle *nh, std::string name, std::string &param, std::string default_param)
{
  nh->param<std::string>(name, param, default_param);
  ROS_INFO("[%s::Config::loadParam::%s]: %s = %s", package_name.c_str(), nh->getNamespace().c_str(), name.c_str(), param.c_str());
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, int &param, int default_param)
{
  nh->param<int>(name, param, default_param);
  ROS_INFO("[%s::Config::loadParam]: %s = %i", package_name.c_str(), name.c_str(), param);
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, double &param, double default_param)
{
  nh->param<double>(name, param, default_param);
  ROS_INFO("[%s::Config::loadParam]: %s = %f", package_name.c_str(), name.c_str(), param);
}

void Config::loadParam(ros::NodeHandle *nh, std::string name, bool &param, bool default_param)
{
  nh->param<bool>(name, param, default_param);
  if (param)
  {
    ROS_INFO("[%s::Config::loadParam]: %s = true", package_name.c_str(), name.c_str());
  }
  else
  {
    ROS_INFO("[%s::Config::loadParam]: %s = false", package_name.c_str(), name.c_str());
  }
}