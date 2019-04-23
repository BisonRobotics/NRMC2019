#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <navigation_msgs/BezierSegment.h>
#include <drive_controller/drive_control_client.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <occupancy_grid/bezier.h>
#include <sstream>
#include <tf2_ros/transform_listener.h>
#include <competition/controller.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_control_client");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  //ros::Subscriber joy_sub = nh.subscribe("joy", 2, joyCallback);
  //ros::Publisher path_pub = nh.advertise<visualization_msgs::MarkerArray>("path_visual", 1);
  //control_point_server = new InteractiveMarkerServer("path_control_points");
/*
  safety = false;
  client = new DriveControlClient;

  path.direction_of_travel = 1;
  path.p0.x = 1.0; path.p0.y = 1.0; path.p0.z = sz / 2;
  path.p1.x = 2.0; path.p1.y = 2.0; path.p1.z = sz / 2;
  path.p2.x = 3.0; path.p2.y = 3.0; path.p2.z = sz / 2;
  path.p3.x = 4.0; path.p3.y = 4.0; path.p3.z = sz / 2;
  size_t path_size = 20;
  MarkerArray path_visual = createPathVisual(path, path_size);
  createControlMarkers(control_point_server);

  while(ros::ok())
  {
    ros::spinOnce();
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    updatePathVisual(&path_visual, path, path_size);
    path_pub.publish(path_visual);
    rate.sleep();
  }*/
}