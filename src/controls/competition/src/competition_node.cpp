#include <ros/ros.h>
#include <competition/competition_controller.h>

using namespace competition;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "competition_controller");
  ros::NodeHandle nh("~");
  Config config(&nh);
  ros::Rate rate(config.rate());
  Controller controller(&nh, config);

  while(ros::ok())
  {
    ros::spinOnce();
    controller.update();
    rate.sleep();
  }
}