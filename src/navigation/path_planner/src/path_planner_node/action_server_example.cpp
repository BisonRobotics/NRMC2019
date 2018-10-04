#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <navigation_msgs/FollowPathAction.h>
#include <navigation_msgs/Point2D.h>
#include <occupancy_grid/bezier.h>

using navigation_msgs::FollowPathAction;
using navigation_msgs::FollowPathGoalConstPtr;
using navigation_msgs::FollowPathFeedback;
using navigation_msgs::FollowPathResult;
using navigation_msgs::BezierSegment;
using actionlib::SimpleActionServer;
using occupancy_grid::Bezier;

SimpleActionServer<FollowPathAction> *server;

void callback(const FollowPathGoalConstPtr &goal)
{
  ROS_INFO("[action_server] Moving toward goal");
  ros::Rate rate(1.0);

  // Get path
  BezierSegment segment = goal->path[0];

  // Provide feedback
  FollowPathFeedback feedback;
  feedback.deviation = 0.01;
  for (int i = 0; i < 10; i++)
  {
    feedback.progress = 0.1 * (i + 1);
    server->publishFeedback(feedback);
    rate.sleep();
  }

  // Publish result
  FollowPathResult result;
  result.pose.position.x = segment.p3.x;
  result.pose.position.y = segment.p3.y;
  result.pose.position.z = 0.0;
  // TODO add orientation
  result.status = 0;
  server->setSucceeded(result);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "action_server_example");
  ros::NodeHandle nh;
  server = new SimpleActionServer<FollowPathAction>(nh, "follow_path", &callback, false);
  server->start();
  ROS_INFO("[action_server] Started");

  ros::spin();

  return 0;
}

