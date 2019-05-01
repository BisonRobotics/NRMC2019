#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H

#include <string>
#include <boost/math/constants/constants.hpp>
#include <ros/ros.h>
#include <waypoint_control/WaypointControlAction.h>
#include <utilities/config.h>

namespace waypoint_control
{
  typedef std::vector<Waypoint> Waypoints;
  using boost::math::double_constants::pi;

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

  class DriveProfile
  {
  public:
    DriveProfile(double max_in_place_duty, double min_in_place_duty,
        double max_driving_duty, double min_driving_duty, double in_place_k,
        double driving_kx, double driving_ky);
    const double &maxInPlaceDuty() const;
    const double &minInPlaceDuty() const;
    const double &maxDrivingDuty() const;
    const double &minDrivingDuty() const;
    const double &inPlaceK() const;
    const double &drivingKx() const;
    const double &drivingKy() const;

  private:
    double max_in_place_duty_;
    double min_in_place_duty_;
    double max_driving_duty_;
    double min_driving_duty_;
    double in_place_k_;
    double driving_kx_;
    double driving_ky_;
  };

  class Config : public utilities::Config
  {
  public:
    Config(ros::NodeHandle *nh);

    const double &dt();
    const double &rate();
    const double &maxAcceleration();
    const double &maxDuty();
    const double &maxManualDuty();
    const double &minManualDuty();
    const bool   &fullAutonomy();
    const bool   &voltageCompensation();
    const double &minVoltage();
    const double &fullVoltage();
    const double &maxCompensatedDuty();
    const double &batteryFilterK();
    const DriveProfile &driveProfile(int i);

  private:
    double dt_;
    double rate_;
    double max_duty_;
    double max_manual_duty_;
    double min_manual_duty_;
    double max_acceleration_;
    bool full_autonomy_;
    bool voltage_compensation_;
    double min_voltage_;
    double full_voltage_;
    double max_compensated_duty_;
    double battery_filter_k_;
    std::vector<DriveProfile> drive_profiles_;
  };
}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROL_STATES_H
