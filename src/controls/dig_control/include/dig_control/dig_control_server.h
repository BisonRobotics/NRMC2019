#ifndef DIG_CONTROL_DIG_CONTROLLER_SERVER_H
#define DIG_CONTROL_DIG_CONTROLLER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dig_control/DigControlAction.h>
#include <dig_control/dig_controller.h>
#include <sensor_msgs/Joy.h>

namespace dig_control
{
  class DigControlServer
  {
  public:
    explicit DigControlServer(ros::NodeHandle *nh, DigController *controller);

    void goalCallback(const actionlib::SimpleActionServer<DigControlAction>::GoalConstPtr &goal);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void preemptCallback();
    void update();

    static DigControlResult toResult(ControlState state);
    static ControlState toControlState(DigControlGoal goal);

  private:
    bool dig_safety;
    float backhoe_duty, bucket_duty, central_duty, vibrator_duty;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    actionlib::SimpleActionServer<dig_control::DigControlAction> server;
    bool active_request;
    DigController *controller;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
