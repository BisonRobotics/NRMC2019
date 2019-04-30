#include <competition/config.h>

#include <ros/ros.h>
#include <competition/competition_controller.h>

using namespace competition;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control_client");
  ros::NodeHandle nh("~");
  ros::Rate rate(50);

  Config config(&nh);

}
