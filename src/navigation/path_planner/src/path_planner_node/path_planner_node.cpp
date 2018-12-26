#include <iostream>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <boost/timer/timer.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <occupancy_grid/arena.h>
#include <occupancy_grid/display.h>
#include <path_planner/path_planner.h>
#include <navigation_msgs/PlanPath.h>
#include <navigation_msgs/BezierSegment.h>
#include <navigation_msgs/FollowPathAction.h>
#include <navigation_msgs/direction.h>
#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid_ros/occupancy_grid_ros.h>
#include <actionlib/client/simple_action_client.h>

using namespace path_planner;
using namespace occupancy_grid;
using namespace cv::viz;
using boost::timer::cpu_timer;
using navigation_msgs::FollowPathAction;
using navigation_msgs::FollowPathGoal;
using navigation_msgs::FollowPathFeedbackConstPtr;
using navigation_msgs::FollowPathResultConstPtr;
using actionlib::SimpleActionClient;
using navigation_msgs::Direction;

occupancy_grid::OccupancyGrid *map;
ros::Publisher *path_publisher;
actionlib::SimpleActionClient<FollowPathAction> *follower_client;
volatile unsigned int count;

void doneCallback(const actionlib::SimpleClientGoalState &state,
                  const FollowPathResultConstPtr &result)
{
  ROS_INFO("[path_planner_node]: Action finished at: %f, %f", result.get()->pose.position.x,
      result.get()->pose.position.y);
  ROS_INFO("[path_planner_node]: Ready for new goal");
}

void activeCallback()
{
  ROS_INFO("[path_planner_node]: Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCallback(const FollowPathFeedbackConstPtr &feedback)
{
  ROS_INFO("[path_planner_node]: Goal Progress: %f", feedback->progress);
}

bool planPath(navigation_msgs::PlanPath::Request &req,
              navigation_msgs::PlanPath::Response &res)
{
  try
  {
    ROS_INFO("[path_planner_node]: Finding a path...");
    cpu_timer timer;
    PathPlanner planner;
    Bezier best_curve = planner.findSegment(*map, Point(req.goals[0].x, req.goals[0].y),
                                                  Point(req.goals[1].x, req.goals[1].y));
    std::cout << "Time:" << timer.format(2);
    std::cout << "Path cost: " << best_curve.path_cost << ", Max cost: " << best_curve.max_cost << std::endl;
    std::cout << "Min radius of curvature: " << best_curve.min_radius(20) << std::endl;

    nav_msgs::Path path_visual;
    convert(best_curve, &path_visual, 20);
    updateHeader(&path_visual, count, ros::Time::now());
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
    segment_1.direction_of_travel = req.goals[1].x > req.goals[0].x ?  //This is temp.
                                    static_cast<int8_t>(Direction::forward) :
                                    static_cast<int8_t>(Direction::reverse);

    res.path.push_back(segment_1);

    ROS_INFO("[path_planner_node]: Sending goal to action server");
    FollowPathGoal goal;
    goal.path = res.path;
    follower_client->sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

    count++;

    return true;
  }
  catch (std::runtime_error &e)
  {
    ROS_WARN("[path_planner_node]: %s", e.what());
    return false;
  }
}

void mapCallback(nav_msgs::OccupancyGrid const &new_map)
{
  ROS_INFO("[path_planner_node]: Received occupancy grid");
  occupancy_grid::convert(new_map, map);
  occupancy_grid::write(*map);
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
  follower_client = new SimpleActionClient<FollowPathAction>("follow_path", true);

  count = 0;
  map = new OccupancyGrid(ArenaDimensions::height_cm, ArenaDimensions::width_cm);

  ROS_INFO("[path_planner_node]: Waiting for action server to start...");
  follower_client->waitForServer();
  ROS_INFO("[path_planner_node]: Action server started, waiting for goal...");

  ros::spin();

  return 0;
}