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
    explicit DigControlServer(ros::NodeHandle *nh, Config config);

    void goalCallback();
    void preemptCallback();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void update();
    void updateCentralDriveAngle();
    double getCentralDriveAngle() const;
    double getMonoBoomAngle() const;
    double getBackhoeAngle() const;
    double getFlapsAngle() const;
    double getBucketAngle() const;
    static double polyFit(const std::vector<double> &p, double x);
    void stop();

    static DigControlResult toResult(ControlState state);
    static ControlState toControlState(DigControlGoal goal);

  private:
    // Minimum 0.22
    // Dump angle 2.12 // Austin has it at 1.3
    // Maximum 2.35
    bool manual_safety, autonomy_safety;
    double backhoe_duty, bucket_duty, central_duty, vibrator_duty;
    uint32_t seq;
    std::vector<double> monoboom_params, flap_params, backhoe_params;
    double central_drive_angle;

    ros::NodeHandle *nh;
    ros::Subscriber joy_subscriber;
    ros::Publisher debug_publisher;
    ros::Publisher joint_publisher;
    actionlib::SimpleActionServer<dig_control::DigControlAction> server;
    sensor_msgs::JointState joint_angles;

    Config config;
    DigController controller;
  };
}

#endif //DIG_CONTROL_DIG_CONTROLLER_SERVER_H
