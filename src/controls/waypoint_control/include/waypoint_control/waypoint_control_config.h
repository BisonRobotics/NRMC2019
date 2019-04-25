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
    const double INITIAL_ANGULAR_VELOCITY = 10.0*pi/180.0;
    const double MAX_VELOCITY = 0.1;
    const double MAX_LINEAR_VELOCITY = 0.1;
    const double MAX_ANGULAR_VELOCITY = 0.05;
  };

  class Config
  {
  public:
    Config();
    Config(ros::NodeHandle *nh);

    double initial_angular_variation;
    double max_velocity;
    double max_angular_velocity;
    double max_linear_velocity;
  };

  enum class ControlState
  {
    error = 0,
    ready,
    new_goal,
    in_progress,
    cancel,
    manual
  };

  enum class WaypointState
  {
    error = 0,
    ready,
    starting_orientation,
    initial_angle_correction,
    driving,
    angle_correction,
    final_angle_correction,
  };

  WaypointControlResult toResult(ControlState state);
  ControlState toControlState(WaypointControlGoal goal);
  std::string to_string(ControlState state);
  std::string to_string(WaypointState state);
}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
