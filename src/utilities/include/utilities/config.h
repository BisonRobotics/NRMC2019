#ifndef UTILITIES_CONFIG_H
#define UTILITIES_CONFIG_H

#include <ros/ros.h>

namespace utilities
{
  class Config
  {
  public:
    Config(std::string package_name);

    void loadParam(ros::NodeHandle *nh, std::string name, std::string &param, std::string default_param);
    void loadParam(ros::NodeHandle *nh, std::string name, bool &param, bool default_param);
    void loadParam(ros::NodeHandle *nh, std::string name, int &param, int default_param);
    void loadParam(ros::NodeHandle *nh, std::string name, double &param, double default_param);

  protected:
    std::string package_name;
  };
}

#endif //UTILITIES_CONFIG_H
