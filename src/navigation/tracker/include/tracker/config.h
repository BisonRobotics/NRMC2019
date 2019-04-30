#ifndef TRACKER_CONFIG_H
#define TRACKER_CONFIG_H

#include <ros/ros.h>
#include <string>
#include <utilities/config.h>

namespace tracker
{
  class Config : utilities::Config
  {
  public:
    Config(ros::NodeHandle *base_nh, ros::NodeHandle *nh, std::string name);

    std::string name;
    double max_initialization_velocity;
    double max_scan_velocity;
    double max_velocity;
    double k;
    double tag_switch_x;
    double tag_switch_y;
    int stepper_controller_id;
    int stepper_client_id;
    int brightness;
    int exposure;
  };
}


#endif //TRACKER_CONFIG_H
