#include <ros/ros.h>
#include <competition/controller.h>

using navigation_msgs::BezierSegment;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control_client");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  BezierSegment path;
  path.direction_of_travel = 1;
  path.p0.x = 1.77; path.p0.y = 3.03;
  path.p1.x = 1.63; path.p1.y = 4.02;
  path.p2.x = 2.94; path.p2.y = 4.67;
  path.p3.x = 2.61; path.p3.y = 6.00;

  waypoint_control::Waypoints waypoints;

  competition::Controller controller(&nh, &rate, waypoints);

  while(ros::ok())
  {
    ros::spinOnce();
    controller.update();
    rate.sleep();
  }
}