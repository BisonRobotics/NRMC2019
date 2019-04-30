#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROL_SERVER_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROL_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/Joy.h>
#include <tf2_ros/transform_listener.h>
#include <navigation_msgs/FollowPathAction.h>
#include <waypoint_control/WaypointControlAction.h>
#include <teleop_interface/teleop_interface.h>
#include <localization/StateVector.h>
#include <waypoint_control/config.h>
#include <waypoint_control/waypoint_controller.h>

namespace waypoint_control
{
  class WaypointControlServer
  {
  public:

    WaypointControlServer(ros::NodeHandle *nh, Config *config,
        iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl);

    void goalCallback();
    void preemptCallback();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void update();

  private:
    bool debug, manual_safety, autonomy_safety;
    uint32_t seq;
    Config *config;
    double dt, teleop_left, teleop_right;
    Waypoints waypoints;

    iVescAccess *fl, *fr, *br, *bl;
    WaypointController controller;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Publisher joint_publisher;
    ros::Publisher debug_publisher;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    actionlib::SimpleActionServer<WaypointControlAction> server;
    sensor_msgs::JointState joint_angles;
  };
}


#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROL_SERVER_H
