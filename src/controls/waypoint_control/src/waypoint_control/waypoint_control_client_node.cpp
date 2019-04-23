#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <navigation_msgs/BezierSegment.h>
#include <waypoint_control/waypoint_control_client.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>
#include <occupancy_grid/bezier.h>
#include <sstream>
#include <tf2_ros/transform_listener.h>

using namespace waypoint_control;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using interactive_markers::InteractiveMarkerServer;
using navigation_msgs::BezierSegment;
using occupancy_grid::Bezier;
using geometry_msgs::Point;
using std::string;
using std::to_string;

bool safety;
const double sz = 0.2, sx = 0.1, st = 0.1;
BezierSegment path;
WaypointControlClient *client;
InteractiveMarkerServer *control_point_server;

void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  bool lb = joy->buttons[4] == 1; // Safety

  safety = lb;
  //ROS_INFO("Safety: %i", lb);

  if (!safety)
  {
    if (client->getControlState() != ControlState::manual)
    {
      client->setControlState(ControlState::manual);
    }
  }
}

void controlMarkerFeedback(const InteractiveMarkerFeedback::ConstPtr &feedback)
{
  string name = feedback->marker_name;
  double x = feedback->pose.position.x;
  double y = feedback->pose.position.y;

  if (name == "p0")
  {
    path.p0.x = x;
    path.p0.y = y;
  }
  else if (name == "p1")
  {
    path.p1.x = x;
    path.p1.y = y;
  }
  else if (name == "p2")
  {
    path.p2.x = x;
    path.p2.y = y;
  }
  else if (name == "p3")
  {
    path.p3.x = x;
    path.p3.y = y;
  }

  InteractiveMarker control_point;
  control_point_server->get(name, control_point);
  std::stringstream ss;
  ss.precision(2);
  ss << std::fixed << name << "(" << x << ", " << y << ")";
  control_point.controls[1].markers[0].text = ss.str();
  control_point_server->insert(control_point);
  control_point_server->applyChanges();
}

InteractiveMarker createControlMarker(std::string name, double x, double y)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = name;
  int_marker.description = "";
  int_marker.scale = 0.4;
  int_marker.pose.position.x = x;
  int_marker.pose.position.y = y;

  Marker marker;
  marker.type = Marker::CYLINDER;
  marker.scale.x = sx;
  marker.scale.y = sx;
  marker.scale.z = sz;
  marker.color.r = 0.3;
  marker.color.g = 0.3;
  marker.color.b = 0.3;
  marker.color.a = 1.0;
  marker.pose.position.z = sz / 2.0;
  InteractiveMarkerControl control;
  control.name = name + "_visual";
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.always_visible = 1;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.markers.emplace_back(marker);
  int_marker.controls.emplace_back(control);

  Marker text_marker;
  text_marker.type = Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.scale.x = st;
  text_marker.scale.y = st;
  text_marker.scale.z = st;
  std::stringstream ss; ss.precision(2);
  ss << std::fixed << name << "("  << x << ", " << y << ")";
  text_marker.text = ss.str();
  text_marker.pose.position.x = -0.1;
  text_marker.pose.position.y = 0.35;
  text_marker.pose.position.z = sz + 0.05;
  InteractiveMarkerControl text_control;
  text_control.always_visible = 1;
  text_control.markers.emplace_back(text_marker);
  int_marker.controls.emplace_back(text_control);

  return int_marker;
}

void createControlMarkers(interactive_markers::InteractiveMarkerServer *server)
{
  InteractiveMarker p0_marker = createControlMarker("p0", path.p0.x, path.p0.y);
  InteractiveMarker p1_marker = createControlMarker("p1", path.p1.x, path.p1.y);
  InteractiveMarker p2_marker = createControlMarker("p2", path.p2.x, path.p2.y);
  InteractiveMarker p3_marker = createControlMarker("p3", path.p3.x, path.p3.y);

  server->insert(p0_marker, &controlMarkerFeedback);
  server->insert(p1_marker, &controlMarkerFeedback);
  server->insert(p2_marker, &controlMarkerFeedback);
  server->insert(p3_marker, &controlMarkerFeedback);

  server->applyChanges();
}

MarkerArray createPathVisual(const BezierSegment &s, size_t path_size)
{
  Marker vertices_marker;
  vertices_marker.id = 0;
  vertices_marker.header.frame_id = "map";
  vertices_marker.header.stamp = ros::Time::now();
  vertices_marker.header.seq++;
  vertices_marker.lifetime = ros::Duration(1.0);
  vertices_marker.ns = "path_vertices";
  vertices_marker.action = Marker::ADD;
  vertices_marker.pose.orientation.w = 1.0;
  vertices_marker.type = Marker::LINE_LIST;
  vertices_marker.scale.x = 0.02;
  vertices_marker.color.r = 0.5;
  vertices_marker.color.g = 0.5;
  vertices_marker.color.b = 0.5;
  vertices_marker.color.a = 1.0;
  vertices_marker.points.resize(4);

  Marker path_marker;
  path_marker.id = 1;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = ros::Time::now();
  path_marker.header.seq++;
  path_marker.lifetime = ros::Duration(1.0);
  path_marker.ns = "path";
  path_marker.action = Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.type = Marker::LINE_STRIP;
  path_marker.scale.x = 0.02;
  path_marker.color.b = 1.0;
  path_marker.color.a = 1.0;
  path_marker.points.resize(path_size + 1);

  MarkerArray path_visual;
  path_visual.markers.emplace_back(vertices_marker);
  path_visual.markers.emplace_back(path_marker);
  return path_visual;
}

void updatePathVisual(MarkerArray *path_visual, const BezierSegment &s, size_t path_size)
{
  // Update vertices
  path_visual->markers[0].points[0] = path.p0;
  path_visual->markers[0].points[1] = path.p1;
  path_visual->markers[0].points[2] = path.p2;
  path_visual->markers[0].points[3] = path.p3;

  // Update path
  Bezier path(s.p0.x, s.p0.y, s.p1.x, s.p1.y, s.p2.x, s.p2.y, s.p3.x, s.p3.y);
  double size_d = path_size;
  occupancy_grid::Point point_2d;
  Point point_3d;
  path_visual->markers[1].points[0] = s.p0;
  for (size_t i = 1; i <= path_size; i++)
  {
    point_2d = path(0, i / size_d);
    point_3d.x = point_2d.x;
    point_3d.y = point_2d.y;
    point_3d.z = sz / 2.0;
    path_visual->markers[1].points[i] = point_3d;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_control_client");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ros::Subscriber joy_sub = nh.subscribe("/joy", 2, joyCallback);
  ros::Publisher path_pub = nh.advertise<visualization_msgs::MarkerArray>("path_visual", 1);
  control_point_server = new InteractiveMarkerServer("path_control_points");

  safety = false;
  client = new WaypointControlClient;

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
  }
}
