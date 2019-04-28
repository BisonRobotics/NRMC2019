#include <competition/waypoint_visuals.h>
#include <eigen3/Eigen/Geometry>
#include <boost/math/constants/constants.hpp>

using namespace competition;
using boost::math::double_constants::pi;

WaypointVisuals::WaypointVisuals(ros::NodeHandle *nh) :
  nh(nh), marker_server("path_control_points"), reverse(false),
  height(0.2), width(0.1), text_size(0.1), precision(2), size(20), seq(0),
  boostControlMarkerFeedback(boost::bind(&WaypointVisuals::controlMarkerFeedback, this, _1))
{
  bool dynamic_waypoints, follow_robot;
  nh->param<bool>("dynamic_waypoints", dynamic_waypoints, true);
  nh->param<bool>("follow_robot", follow_robot, true);
  dynamicWaypoints(dynamic_waypoints);
  followRobot(follow_robot);

  addControlMarker("robot", 1.0, 1.0);
  marker_server.applyChanges();

  waypoints_pub = nh->advertise<visualization_msgs::MarkerArray>("waypoint_visuals", 1);
  add_waypoint_sub = nh->subscribe("/clicked_point", 10, &WaypointVisuals::addWaypoint, this);
}

void WaypointVisuals::addWaypoint(const geometry_msgs::PointStampedConstPtr &point)
{
  Waypoint waypoint;
  waypoint.header.frame_id = "P" + std::to_string(seq);
  waypoint.header.seq = seq;
  waypoint.header.stamp = ros::Time::now();
  waypoint.pose.x = point->point.x;
  waypoint.pose.y = point->point.y;
  waypoint.reverse = (uint8_t)reverse;
  waypoints.emplace_back(waypoint);
  double x = waypoint.pose.x;
  double y = waypoint.pose.y;
  addControlMarker(waypoint.header.frame_id, x, y);
  marker_server.applyChanges();
  updateWaypointVisuals(waypoints, robot);
  ROS_INFO("[WaypointVisuals::addWaypoint]: Added waypoint at (%4f, %4f)", x, y);
}

void WaypointVisuals::updateWaypoints(const Waypoints &s)
{
  seq = 0;
  for (auto waypoint = s.begin(); waypoint != s.end(); ++waypoint)
  {
    waypoints.emplace_back(*waypoint);
    addControlMarker(waypoint->header.frame_id, waypoint->pose.x, waypoint->pose.y);
    seq = waypoint->header.seq;
  }
  marker_server.applyChanges();
  updateWaypointVisuals(waypoints, robot);
}

void WaypointVisuals::update(tf2::Transform robot)
{
  this->robot = robot;
  if (follow_robot)
  {
    double x0 = robot.getOrigin().x();
    double y0 = robot.getOrigin().y();
    InteractiveMarker control_point;
    if (marker_server.get("robot", control_point))
    {
      control_point.pose.position.x = x0;
      control_point.pose.position.y = y0;
      //control_point.controls[0].markers[0].scale.x = 0.0001;
      //control_point.controls[0].markers[0].scale.y = 0.0001;
      //control_point.controls[0].markers[0].scale.z = 0.0001;
      control_point.controls[0].interaction_mode = InteractiveMarkerControl::NONE;
      marker_server.insert(control_point, boostControlMarkerFeedback);
      marker_server.applyChanges();
    }
    else
    {
      ROS_ERROR("[WaypointVisuals::update]: Unable to find robot marker");
    }
  }

  updateWaypointVisuals(waypoints, robot);
}

void WaypointVisuals::controlMarkerFeedback(const InteractiveMarkerFeedback::ConstPtr &feedback)
{
  std::string name = feedback->marker_name;
  /*double x = feedback->pose.position.x;
  double y = feedback->pose.position.y;
  InteractiveMarker control_point;
  marker_server.get(name, control_point);
  control_point.controls[1].markers[0].text = markerString(name, x, y);
  marker_server.insert(control_point, boostControlMarkerFeedback);
  marker_server.applyChanges();*/
  bool found = false;
  for (int i = 0; i < waypoints.size(); i++)
  {
    if (feedback->marker_name == waypoints[i].header.frame_id)
    {
      waypoints[i].pose.x = feedback->pose.position.x;
      waypoints[i].pose.y = feedback->pose.position.y;
      found = true;
    }
  }
  if (!found)
  {
    ROS_ERROR("[WaypointVisuals::controlMarkerFeedback]: Unable to find waypoint");
  }
}

void WaypointVisuals::addControlMarker(std::string name, double x, double y)
{
  InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = "map";
  interactive_marker.header.stamp = ros::Time::now();
  interactive_marker.name = name;
  interactive_marker.description = "";
  interactive_marker.scale = 0.4;
  interactive_marker.pose.position.x = x;
  interactive_marker.pose.position.y = y;

  Marker marker;
  marker.type = Marker::CYLINDER;
  marker.scale.x = width;
  marker.scale.y = width;
  marker.scale.z = height;
  marker.color.r = 0.3;
  marker.color.g = 0.3;
  marker.color.b = 0.3;
  marker.color.a = 1.0;
  marker.pose.position.z = height / 2.0;
  InteractiveMarkerControl control;
  control.name = name + "_visual";
  tf2::Quaternion q(1, 0, 1, 0);
  q = q.normalize();
  control.orientation.w = q.w();
  control.orientation.x = q.x();
  control.orientation.y = q.y();
  control.orientation.z = q.z();
  control.always_visible = 1;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.markers.emplace_back(marker);
  interactive_marker.controls.emplace_back(control);

  marker_server.insert(interactive_marker, boostControlMarkerFeedback);
}

void WaypointVisuals::updateWaypointVisuals(const Waypoints &s, const tf2::Transform &robot_transform)
{
  MarkerArray marker_visuals;

  // Create robot visual
  double x = robot_transform.getOrigin().x();
  double y = robot_transform.getOrigin().y();
  tf2::Matrix3x3 R(robot.getRotation());
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);
  Eigen::Rotation2D<double> theta(yaw);
  Marker text_marker;
  text_marker.id = 0;
  text_marker.header.seq = seq++;
  text_marker.header.stamp = ros::Time::now();
  text_marker.header.frame_id = "map";
  text_marker.type = Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.scale.x = text_size;
  text_marker.scale.y = text_size;
  text_marker.scale.z = text_size;
  text_marker.text = markerString("robot", x, y, theta.smallestAngle());
  text_marker.pose.position.x = x;
  text_marker.pose.position.y = y;
  text_marker.pose.position.z = 1.0;
  text_marker.pose.orientation.w = 1.0;
  text_marker.lifetime = ros::Duration(1.0);
  marker_visuals.markers.emplace_back(text_marker);

  // Create waypoints visuals
  Marker waypoints_marker;
  waypoints_marker.id = 1;
  waypoints_marker.header.frame_id = "map";
  waypoints_marker.header.stamp = ros::Time::now();
  waypoints_marker.header.seq++;
  waypoints_marker.lifetime = ros::Duration(1.0);
  waypoints_marker.ns = "path";
  waypoints_marker.action = Marker::ADD;
  waypoints_marker.pose.orientation.w = 1.0;
  waypoints_marker.type = Marker::LINE_STRIP;
  waypoints_marker.scale.x = 0.02;
  waypoints_marker.color.r = 0.01;
  waypoints_marker.color.g = 0.01;
  waypoints_marker.color.b = 1.0;
  waypoints_marker.color.a = 1.0;
  waypoints_marker.lifetime = ros::Duration(1.0);

  geometry_msgs::Point robot_point = text_marker.pose.position;
  robot_point.z = height / 2.0;
  waypoints_marker.points.emplace_back(robot_point);
  for (int i = 0; i < s.size(); i++)
  {
    double x = waypoints[i].pose.x;
    double y = waypoints[i].pose.y;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = height / 2.0;
    waypoints_marker.points.emplace_back(p);
    Marker text_marker;
    text_marker.id = waypoints[i].header.seq;
    text_marker.header.seq = waypoints[i].header.seq;
    text_marker.header.stamp = ros::Time::now();
    text_marker.header.frame_id = "map";
    text_marker.type = Marker::TEXT_VIEW_FACING;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.scale.x = text_size;
    text_marker.scale.y = text_size;
    text_marker.scale.z = text_size;
    text_marker.text = markerString("P" + std::to_string(i), x, y);
    text_marker.pose.position.x = x;
    text_marker.pose.position.y = y;
    text_marker.pose.position.z = height + 0.15;
    text_marker.lifetime = ros::Duration(1.0);
    marker_visuals.markers.emplace_back(text_marker);
  }
  marker_visuals.markers.emplace_back(waypoints_marker);

  waypoints_pub.publish(marker_visuals);
}

std::string WaypointVisuals::markerString(const std::string &name, double x, double y)
{
  std::stringstream ss;
  ss.precision(precision);
  ss << std::fixed << name << "("  << x << ", " << y << ")";
  return ss.str();
}

std::string WaypointVisuals::markerString(const std::string &name, double x, double y, double theta)
{
  std::stringstream ss;
  ss.precision(precision);
  ss << std::fixed << name << "("  << x << ", " << y << ", " << theta / pi * 180 << ")";
  return ss.str();
}

void WaypointVisuals::followRobot(bool follow)
{
  InteractiveMarker control_point;
  if (!follow)
  {
    if (marker_server.get("robot", control_point))
    {
      control_point.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
      marker_server.insert(control_point, boostControlMarkerFeedback);
      marker_server.applyChanges();
    }
    else
    {
      ROS_ERROR("[WaypointVisuals::update]: Unable to find robot marker");
    }
  }
  this->follow_robot = follow;
}

void WaypointVisuals::dynamicWaypoints(bool dynamic_waypoints)
{
  this->dynamic_waypoints = dynamic_waypoints;
}

void WaypointVisuals::clearWaypoints()
{
  waypoints.clear();
  marker_server.clear();
  addControlMarker("robot", robot.getOrigin().x(), robot.getOrigin().y());
  marker_server.applyChanges();
}

Waypoints WaypointVisuals::getWaypoints()
{
  return waypoints;
}

void WaypointVisuals::setReverse(bool reverse)
{
  this->reverse = reverse;
}
