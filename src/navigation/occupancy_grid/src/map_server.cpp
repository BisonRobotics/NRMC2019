#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>

#include <occupancy_grid/arena.h>
#include <occupancy_grid_ros/occupancy_grid_ros.h>
#include <navigation_msgs/ObstacleLayout.h>

using namespace occupancy_grid;
using cv::viz::Color;

std::vector<Circle> *rocks;
Arena *arena;
unsigned int count = 0;

ros::Publisher *map_publisher;

bool obstacleCallback(navigation_msgs::ObstacleLayout::Request &req,
                     navigation_msgs::ObstacleLayout::Response &res)
{
  rocks->clear();
  for (auto rock = req.obstacles.begin(); rock != req.obstacles.end(); rock++)
  {
    rocks->emplace_back(rock->p.x, rock->p.y, rock->r);
  }
  arena->~Arena();
  arena = new Arena(*rocks);
  nav_msgs::OccupancyGrid map;
  convert(arena->inflated_obstacles, &map);
  ros::Time generated_at = ros::Time::now();
  updateHeader(&map, count++, ros::Time::now(), generated_at);
  map_publisher->publish(map);
}

int main(int argc, char **argv)
{
  using AD = ArenaDimensions;

  ros::init(argc, argv, "map_server");
  ros::NodeHandle nh;
  map_publisher = new ros::Publisher;
  (*map_publisher) = nh.advertise<nav_msgs::OccupancyGrid>("inflated_obstacles", 1, true);
  ros::ServiceServer obstacle_service = nh.advertiseService("obstacle_config", obstacleCallback);
  ros::Rate loop_rate(10);

  rocks = new std::vector<Circle>();
  rocks->emplace_back(3.0,  1.0, 0.3);
  rocks->emplace_back(4.0, -0.5, 0.3);
  rocks->emplace_back(2.5, -0.9, 0.3);
  arena = new Arena(*rocks);

  nav_msgs::OccupancyGrid map;
  convert(arena->inflated_obstacles, &map);
  ros::Time generated_at = ros::Time::now();
  updateHeader(&map, count++, ros::Time::now(), generated_at);
  map_publisher->publish(map);

  ros::spin();

  return 0;
}