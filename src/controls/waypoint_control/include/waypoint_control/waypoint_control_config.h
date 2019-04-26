#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H

#include <string>
#include <boost/math/constants/constants.hpp>

#include <ros/ros.h>

#include <waypoint_control/WaypointControlAction.h>


namespace waypoint_control
{
  typedef std::vector<Waypoint> Waypoints;
  using boost::math::double_constants::pi;

  class Config
  {
  public:
    Config(ros::NodeHandle *nh);
    static void loadParam(ros::NodeHandle *nh, const std::string &name, double &param, double default_param);

    double rate;
    double max_acceleration;
    double max_duty;
    double max_in_place_duty;
    double min_in_place_duty;
    double max_driving_duty;
    double min_driving_duty;
    double max_manual_duty;
    double min_manual_duty;
    double in_place_k;
    double driving_kx;
    double driving_ky;
  };

  enum class ControlState
  {
    error = 0,
    ready,
    new_goal,
    in_progress,
    finished,
    cancel,
    manual
  };

  enum class WaypointState
  {
    error = 0,
    ready,
    initial_angle_correction,
    driving,
    angle_correction,
    final_angle_correction,
    finished,
  };

  WaypointControlResult toResult(ControlState state);
  ControlState toControlState(WaypointControlGoal goal);
  std::string to_string(ControlState state);
  std::string to_string(WaypointState state);
}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
