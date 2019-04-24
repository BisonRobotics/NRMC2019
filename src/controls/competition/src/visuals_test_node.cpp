#include <ros/ros.h>
#include <competition/waypoint_visuals.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

using namespace competition;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visuals_test_node");
  ros::NodeHandle nh;
  ros::Rate rate(50);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  Waypoints waypoints;

  WaypointVisuals visuals(&nh);
  tf2::Stamped<tf2::Transform> transform;

  while(ros::ok())
  {
    ros::spinOnce();
    try
    {
      tf2::fromMsg(tf_buffer.lookupTransform("map", "base_link", ros::Time(0)), transform);
      visuals.update(transform);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    visuals.update(transform);
    rate.sleep();
  }
}