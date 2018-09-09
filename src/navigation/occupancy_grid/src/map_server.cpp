#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>

#include <occupancy_grid/arena.h>
#include <occupancy_grid_ros/occupancy_grid_ros.h>

using namespace occupancy_grid;
using cv::viz::Color;

int main(int argc, char **argv)
{
  using AD = ArenaDimensions;

  ros::init(argc, argv, "map_server");
  ros::NodeHandle n;
  ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("inflated_obstacles", 10);
  ros::Rate loop_rate(1);

  std::vector<Circle> rocks;
  rocks.emplace_back(3.0,  1.0, 0.3);
  rocks.emplace_back(4.0, -0.5, 0.3);
  rocks.emplace_back(2.5, -0.9, 0.3);
  Arena arena(rocks);

  Image obstacles_layer(AD::height_cm, AD::width_cm, Color::white());
  convert(&obstacles_layer, arena.inflated_obstacles);

  nav_msgs::OccupancyGrid test;
  convert(arena.inflated_obstacles, &test);
  ros::Time generated_at = ros::Time::now();


  int count = 0;
  while (ros::ok())
  {
    updateHeader(&test, count, ros::Time::now(), generated_at);
    map_publisher.publish(test);

    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }


  return 0;
}