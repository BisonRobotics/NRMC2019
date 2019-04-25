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

  namespace default_config
  {
    const double MAX_DUTY = 0.3;
    const double MAX_IN_PLACE_DUTY = 0.2;
    const double MIN_IN_PLACE_DUTY = 0.1;
    const double MAX_DRIVING_DUTY = 0.4;
    const double MIN_DRIVING_DUTY = 0.2;
    const double MAX_MANUAL_DUTY = 0.2;
    const double MIN_MANUAL_DUTY = 0.05;
    const double IN_PLACE_K = 1.0;
    const double DRIVING_K = 100;
  };

  class Config
  {
  public:
    Config();
    Config(ros::NodeHandle *nh);

    double max_duty;
    double max_in_place_duty;
    double min_in_place_duty;
    double max_driving_duty;
    double min_driving_duty;
    double max_manual_duty;
    double min_manual_duty;
    double in_place_k;
    double driving_k;
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
