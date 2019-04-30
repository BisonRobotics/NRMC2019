#ifndef COMPETITION_COMPETITION_CONFIG_H
#define COMPETITION_COMPETITION_CONFIG_H

#include <ros/ros.h>
#include <string>
#include <waypoint_control/Waypoint.h>
#include <utilities/config.h>

namespace competition
{
  enum class ControlState
  {
    manual = 0,
    assisted_autonomy,
    wait_for_start,
    start,
    check_for_apriltag,
    wait_for_localization,
    navigate_to_dig_zone_1,
    dig_1,
    finish_dig_1,
    navigate_to_hopper_1,
    dump_1,
    navigate_to_dig_zone_2,
    dig_2,
    finish_dig_2,
    navigate_to_hopper_2,
    dump_2,
    navigate_to_final_position,
    finished,
  };

  using waypoint_control::Waypoint;
  using Waypoints = std::vector<Waypoint>;

  std::string to_string(ControlState state);

  class Config : public utilities::Config
  {
  public:
    Config(ros::NodeHandle *nh);

    const ros::Rate &rate();
    const double &dt();
    const bool &fullAutonomy();
    const ros::Duration &finishDig1Time();
    const ros::Duration &finishDig2Time();
    const ros::Duration &finishTime();
    const Waypoints &digPath1();
    const Waypoints &digPath2();
    const Waypoints &hopperPath1();
    const Waypoints &hopperPath2();
    const Waypoints &finalPosition();

  private:
    ros::Rate rate_;
    double dt_;
    bool full_autonomy_;
    ros::Duration finish_dig_1_time_, finish_dig_2_time_, finish_time_;
    Waypoints dig_path_1_, dig_path_2_, hopper_path_1_, hopper_path_2_, final_position_;


  };


}

#endif //COMPETITION_COMPETITION_CONFIG_H
