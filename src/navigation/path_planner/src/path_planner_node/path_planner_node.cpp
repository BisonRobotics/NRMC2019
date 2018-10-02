#include <iostream>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <boost/timer/timer.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <occupancy_grid/arena.h>
#include <occupancy_grid/display.h>
#include <path_planner/path_planner.h>
#include <navigation_msgs/Path.h>
#include <navigation_msgs/BezierSegment.h>
#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid_ros/occupancy_grid_ros.h>
#include <nav_msgs/Path.h>


using namespace path_planner;
using namespace occupancy_grid;
using namespace cv::viz;
using boost::timer::cpu_timer;

occupancy_grid::OccupancyGrid *map;
ros::Publisher *path_publisher;
unsigned int count;

bool planPath(navigation_msgs::Path::Request &req,
              navigation_msgs::Path::Response &res)
{
  try
  {
    cpu_timer timer;
    PathPlanner planner;
    Bezier best_curve = planner.findSegment(*map, Point(req.goals[0].p.x, req.goals[0].p.y),
                                                  Point(req.goals[1].p.x, req.goals[1].p.y));
    std::cout << "Time:" << timer.format(2) << std::endl;
    std::cout << "Path cost: " << best_curve.path_cost << ", Max cost: " << best_curve.max_cost << std::endl;
    std::cout << "Min radius of curvature: " << best_curve.min_radius(20) << std::endl;

    nav_msgs::Path path_visual;
    convert(best_curve, &path_visual, 20);
    updateHeader(&path_visual, count++, ros::Time::now());
    path_publisher->publish(path_visual);

    navigation_msgs::BezierSegment segment_1;
    segment_1.p0.x = best_curve.p0.x;
    segment_1.p0.y = best_curve.p0.y;
    segment_1.p1.x = best_curve.p1.x;
    segment_1.p1.y = best_curve.p1.y;
    segment_1.p2.x = best_curve.p2.x;
    segment_1.p2.y = best_curve.p2.y;
    segment_1.p3.x = best_curve.p3.x;
    segment_1.p3.y = best_curve.p3.y;
    segment_1.path_cost = best_curve.path_cost;
    segment_1.min_radius = best_curve.min_radius(20);

    res.path.push_back(segment_1);
    res.status = 0;

    return true;
  }
  catch (std::runtime_error &e)
  {
    std::cerr << e.what() << std::endl;
    return false;
  }
}

void mapCallback(nav_msgs::OccupancyGrid const &new_map)
{
  occupancy_grid::convert(new_map, map);
}

int main( int argc, char** argv )
{
  using AD = ArenaDimensions;

  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle nh;
  ros::Subscriber map_subscriber = nh.subscribe("inflated_obstacles", 1, mapCallback);
  ros::ServiceServer planner_service = nh.advertiseService("plan_path", planPath);
  path_publisher = new ros::Publisher;
  (*path_publisher) = nh.advertise<nav_msgs::Path>("path_visual", 1, true);
  count = 0;

  map = new OccupancyGrid(ArenaDimensions::height_cm, ArenaDimensions::width_cm);

  ros::spin();

  return 0;
}