#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>

#include <utilities/filter.h>
#include <waypoint_control/config.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen3/Eigen/Geometry>
#include <waypoint_control/feedback.h>
#include <waypoint_control/Debug.h>


namespace waypoint_control
{
  using Rotation2D = Eigen::Rotation2D<double>;
  using Pose2D = geometry_msgs::Pose2D;

  double clampAcceleration(double value, double last_value, double limit, double dt);

  class WaypointController
  {
  public:
    WaypointController(Config config, iVescAccess *front_left,   iVescAccess *front_right,
                       iVescAccess *back_right, iVescAccess *back_left);

    void update(const Pose2D& pose, bool manual_safety, bool autonomy_safety, double left, double right);
    void updateControls(const Pose2D& pose);
    void updateBatteryVoltage();
    void setPoint(double left, double right, bool reverse = false);
    void setControlState(ControlState goal);
    void setControlState(ControlState goal, const Waypoints &waypoint);
    size_t remainingWaypoints();
    void stop();
    ControlState getControlState() const;
    Debug getDebugInfo() const;



  private:
    double last_left, last_right, dt, battery_voltage, last_battery_voltage;
    Pose2D last_pose;
    Config config;
    Debug debug_info;
    iVescAccess *fl, *fr, *br, *bl;
    ControlState state;
    WaypointState waypoint_state;
    Waypoints waypoints;
    Feedback feedback, last_feedback;

  };

}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
