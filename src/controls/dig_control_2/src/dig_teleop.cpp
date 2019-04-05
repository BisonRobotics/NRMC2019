#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dig_control_2/dig_controller/dig_controller.h>

using namespace dig_control_2;

float left = 0.0f;
float right = 0.0f;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[4])
  {
    left = joy->axes[1] * 0.2f;
    right = joy->axes[4] * 0.2f;
  }
  else if (joy->buttons[5]) // Boostish mode
  {
    left = joy->axes[1] * 0.4f;
    right = joy->axes[4] * 0.4f;
  }
  else
  {
    left = 0.0f;
    right = 0.0f;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dig_teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 2, callback);
  ros::Rate rate(50);

  DigController dig_controller;
  dig_controller.setGoal(DigController::Goal::manual);

  while (ros::ok())
  {
    dig_controller.update();
    dig_controller.setCentralDriveDuty(left);
    dig_controller.setBackhoeDuty(right);
    rate.sleep();
    ros::spinOnce();
  }
}
