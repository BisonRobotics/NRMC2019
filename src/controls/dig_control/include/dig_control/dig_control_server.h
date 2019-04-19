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
    static double getPolyfit(double *params, double angle);

    static DigControlResult toResult(ControlState state);
    static ControlState toControlState(DigControlGoal goal);

  private:
    static constexpr double monoboom_params[] = {-.0808, -0.0073,  0.0462,  0.9498,  -0.0029};
    static constexpr double flap_params[] = {85.0010, -376.8576, 620.7329, -453.8172, 126.0475};

    bool dig_safety;
    float backhoe_duty, bucket_duty, central_duty, vibrator_duty;
    uint32_t seq;
    bool debug;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Publisher debug_publisher;
    ros::Publisher joint_publisher;
    actionlib::SimpleActionServer<dig_control::DigControlAction> server;

    DigControllerInterface *controller;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
