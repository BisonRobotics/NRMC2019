#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <teleop_interface/teleop_interface.h>

float left = 0.0f;
float right = 0.0f;

void callback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->buttons[4])
  {
    left = joy->axes[1];
    right = joy->axes[4];
  }
  else
  {
    left = 0.0f;
    right = 0.0f;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop");
  ros::NodeHandle n;
  ros::Subscriber joy_sub = n.subscribe("joy", 2, callback);
  ros::Rate rate(50);
  TeleopInterface teleop(TeleopInterface::duty, 0.2f);

  while (ros::ok())
  {
    teleop.update(left, right);
    rate.sleep();
    ros::spinOnce();
  }
}
