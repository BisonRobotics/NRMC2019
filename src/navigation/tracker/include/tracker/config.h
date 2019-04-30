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

    const std::string &name();
    const double &initialScanVelocity();
    const double &maxInitializationVelocity();
    const double &maxScanVelocity();
    const double &maxVelocity();
    const double &k();
    const double &tagSwitchX();
    const double &tagSwitchY();
    const int &stepperControllerID();
    const int &stepperClientID();
    const int &brightness();
    const int &exposure();

  private:
    std::string name_;
    double max_initialization_velocity_;
    double initial_scan_velocity_;
    double max_scan_velocity_;
    double max_velocity_;
    double k_;
    double tag_switch_x_;
    double tag_switch_y_;
    int stepper_controller_id_;
    int stepper_client_id_;
    int brightness_;
    int exposure_;
  };
}


#endif //TRACKER_CONFIG_H
