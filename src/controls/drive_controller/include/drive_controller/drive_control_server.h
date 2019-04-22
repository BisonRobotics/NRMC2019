#ifndef DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H
#define DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/Joy.h>
#include <drive_controller/drive_controller.h>
#include <navigation_msgs/FollowPathAction.h>
#include <drive_controller/DriveControlAction.h>
#include <teleop_interface/teleop_interface.h>

namespace drive_controller
{
  enum class ControlState
  {
    error = 0,
    ready,
    new_goal,
    in_progress,
    cancel,
    manual
  };

  DriveControlResult toResult(ControlState state);
  ControlState toControlState(DriveControlGoal goal);
  std::string to_string(ControlState state);

  class DriveControlServer
  {
  public:
    DriveControlServer(ros::NodeHandle *nh, iVescAccess *fl, iVescAccess *fr, iVescAccess *br, iVescAccess *bl);

    void goalCallback();
    void preemptCallback();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void update(double dt);
    void stop();

  private:
    bool debug, safety;
    uint32_t seq;
    float teleop_left, teleop_right, max_velocity;

    ControlState state;
    iVescAccess *fl, *fr, *br, *bl;
    TeleopInterface teleop;
    DriveController controller;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Publisher debug_publisher;
    ros::Publisher joint_publisher;
    actionlib::SimpleActionServer<DriveControlAction> server;
    sensor_msgs::JointState joint_angles;

  };
}


#endif //DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H
