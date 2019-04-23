#include <ros/ros.h>
#include <competition/controller.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control_client");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  competition::Controller controller(&nh, &rate);

  while(ros::ok())
  {
    ros::spinOnce();
    controller.update();
    rate.sleep();
  }
}