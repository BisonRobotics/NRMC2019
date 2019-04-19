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
    explicit DigControlServer(ros::NodeHandle *nh, DigControllerInterface *controller);

    void goalCallback();
    void preemptCallback();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void update();

    static DigControlResult toResult(ControlState state);
    static ControlState toControlState(DigControlGoal goal);

  private:
    bool dig_safety;
    float backhoe_duty, bucket_duty, central_duty, vibrator_duty;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Publisher debug_publisher;
    actionlib::SimpleActionServer<dig_control::DigControlAction> server;
    uint32_t seq;
    bool debug;
    DigControllerInterface *controller;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
