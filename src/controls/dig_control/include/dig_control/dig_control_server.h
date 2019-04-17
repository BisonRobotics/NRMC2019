#ifndef DIG_CONTROL_DIG_CONTROLLER_SERVER_H
#define DIG_CONTROL_DIG_CONTROLLER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dig_control/DigAction.h>
#include <dig_control/DumpAction.h>

namespace dig_control
{
  class DigControllerServer
  {
  public:
    DigControllerServer(ros::NodeHandle *nh);

  private:
    actionlib::SimpleActionServer<dig_control::DumpAction> dump_server;
    actionlib::SimpleActionServer<dig_control::DigAction> dig_server;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
