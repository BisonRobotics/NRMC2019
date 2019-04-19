#ifndef DIG_CONTROL_DIG_CONTROLLER_SERVER_H
#define DIG_CONTROL_DIG_CONTROLLER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dig_control/DigControlAction.h>
#include <dig_control/dig_controller.h>

namespace dig_control
{
  class DigControlServer
  {
  public:
    explicit DigControlServer(ros::NodeHandle *nh, DigController *controller);

    void goalCallback(const actionlib::SimpleActionServer<DigControlAction>::GoalConstPtr &goal);
    void preemptCallback();

    static DigControlResult toResult(ControlState state);
    static ControlState toControlState(DigControlGoal goal);

  private:
    ros::NodeHandle *nh;
    actionlib::SimpleActionServer<dig_control::DigControlAction> server;
    bool active_request;
    DigController *controller;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
