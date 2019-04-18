#ifndef DIG_CONTROL_DIG_CONTROLLER_SERVER_H
#define DIG_CONTROL_DIG_CONTROLLER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dig_control/DigAction.h>
#include <dig_control/DumpAction.h>

namespace dig_control
{
  class DigControlServer
  {
  public:
    explicit DigControlServer(ros::NodeHandle *nh);

    void digCallback(const actionlib::SimpleActionServer<DigAction>::GoalConstPtr &goal);
    void dumpCallback(const actionlib::SimpleActionServer<DumpAction>::GoalConstPtr &goal);

  private:
    ros::NodeHandle *nh;
    actionlib::SimpleActionServer<dig_control::DigAction> dig_server;
    actionlib::SimpleActionServer<dig_control::DumpAction> dump_server;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
