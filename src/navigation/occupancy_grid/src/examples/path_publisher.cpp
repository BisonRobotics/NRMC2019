#include <ros/ros.h>

#include <occupancy_grid/arena.h>
#include <occupancy_grid_ros/occupancy_grid_ros.h>

using namespace occupancy_grid;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_publisher");
  ros::NodeHandle n;
  ros::Publisher path_publisher = n.advertise<nav_msgs::Path>("path", 10);
  ros::Rate loop_rate(1.0);

  Point start(0.75, -0.75), control1(4.0, 1.0), control2(6.5, -0.4), finish(6.5, 0.8);
  Bezier curve(start, control1, control2, finish);
  nav_msgs::Path path;
  convert(curve, &path, 20);

  int count = 0;
  while (ros::ok())
  {
    updateHeader(&path, count++, ros::Time::now());
    path_publisher.publish(path);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}