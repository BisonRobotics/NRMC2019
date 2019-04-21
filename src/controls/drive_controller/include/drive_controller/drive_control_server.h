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
    DriveControlServer(ros::NodeHandle *nh, DriveController *controller, TeleopInterface *telop);

    void goalCallback();
    void preemptCallback();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void update();
    void stop();

  private:
    bool safety;
    float teleop_left, teleop_right;
    uint32_t seq;
    bool debug;

    ControlState state;
    TeleopInterface *teleop;
    DriveController *controller;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Publisher debug_publisher;
    ros::Publisher joint_publisher;
    actionlib::SimpleActionServer<DriveControlAction> server;

  };
}


#endif //DRIVE_CONTROLLER_DRIVE_CONTROL_SERVER_H
