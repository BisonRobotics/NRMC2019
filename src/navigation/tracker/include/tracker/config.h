#ifndef TRACKER_CONFIG_H
#define TRACKER_CONFIG_H

#include <ros/ros.h>
#include <string>

namespace tracker
{
  class Config
  {
  public:
    Config(ros::NodeHandle *base_nh, ros::NodeHandle *nh, std::string name);

    void loadParam(ros::NodeHandle *nh, std::string name, std::string &param, std::string default_param);
    void loadParam(ros::NodeHandle *nh, std::string name, int &param, int default_param);
    void loadParam(ros::NodeHandle *nh, std::string name, double &param, double default_param);

    std::string name;
    double max_initialization_velocity;
    double max_scan_velocity;
    double max_velocity;
    double k;
    int stepper_controller_id;
    int stepper_client_id;
    int brightness;
    int exposure;
  };
}


#endif //TRACKER_CONFIG_H
