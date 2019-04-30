#ifndef COMPETITION_CONTROLLER_H
#define COMPETITION_CONTROLLER_H

#include <ros/ros.h>
#include <utilities/joy.h>
#include <competition/waypoint_visuals.h>
#include <waypoint_control/waypoint_control_client.h>
#include <dig_control/dig_control_client.h>
#include <competition/config.h>
#include <competition/Debug.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

namespace competition
{
  using WaypointControlState = waypoint_control::ControlState;
  using DigControlState = dig_control::ControlState;
  using utilities::Joy;
  using waypoint_control::Waypoints;

  class Controller
  {
  public:
    Controller(ros::NodeHandle *nh, Config config);

    void update();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  private:
    waypoint_control::WaypointControlClient waypoint_client;
    dig_control::DigControlClient dig_client;
    Joy joy;
    ControlState state;
    WaypointVisuals visuals;
    Waypoints active_waypoints;
    Debug debug;

    ros::NodeHandle *nh;
    Config config;
    ros::Time start_time;

    ros::Subscriber joy_subscriber;
    ros::Publisher debug_publisher;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2::Stamped<tf2::Transform> transform;
  };
}

#endif //COMPETITION_CONTROLLER_H
