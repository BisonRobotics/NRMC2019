#include <competition/visuals.h>

using namespace competition;

Visuals::Visuals(ros::NodeHandle *nh) :
  nh(nh), marker_server("path_control_points"),
  height(0.2), width(0.1), text_size(0.1), precision(2), size(20),
  boostControlMarkerFeedback(boost::bind(&Visuals::controlMarkerFeedback, this, _1))
{
  // Dump default
  path.direction_of_travel = 1;
  path.p0.x = 1.0; path.p0.y = 1.0; path.p0.z = height / 2;
  path.p1.x = 2.0; path.p1.y = 2.0; path.p1.z = height / 2;
  path.p2.x = 3.0; path.p2.y = 3.0; path.p2.z = height / 2;
  path.p3.x = 4.0; path.p3.y = 4.0; path.p3.z = height / 2;

  marker_pub = nh->advertise<visualization_msgs::MarkerArray>("path_visual", 1);
  path_visual = createPathVisual(path, size);
  createControlMarkers(&marker_server);

}

void Visuals::update()
{
  updatePathVisual(&path_visual, path, size);
  marker_pub.publish(path_visual);
}

void Visuals::update(tf2::Transform transform)
{
  ROS_WARN("Not implemented");
}

void Visuals::controlMarkerFeedback(const InteractiveMarkerFeedback::ConstPtr &feedback)
{
  std::string name = feedback->marker_name;
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
  marker_server.get(name, control_point);
  control_point.controls[1].markers[0].text = markerString(name, x, y);
  marker_server.insert(control_point);
  marker_server.applyChanges();
}

visualization_msgs::InteractiveMarker Visuals::createControlMarker(std::string name, double x, double y)
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
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.always_visible = 1;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.markers.emplace_back(marker);
  interactive_marker.controls.emplace_back(control);

  Marker text_marker;
  text_marker.type = Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.scale.x = text_size;
  text_marker.scale.y = text_size;
  text_marker.scale.z = text_size;
  text_marker.text = markerString(name, x, y);
  text_marker.pose.position.x = -0.1;
  text_marker.pose.position.y = 0.35;
  text_marker.pose.position.z = height + 0.05;
  InteractiveMarkerControl text_control;
  text_control.always_visible = 1;
  text_control.markers.emplace_back(text_marker);
  interactive_marker.controls.emplace_back(text_control);

  return interactive_marker;
}

void Visuals::createControlMarkers(InteractiveMarkerServer *server)
{
  InteractiveMarker p0_marker = createControlMarker("p0", path.p0.x, path.p0.y);
  InteractiveMarker p1_marker = createControlMarker("p1", path.p1.x, path.p1.y);
  InteractiveMarker p2_marker = createControlMarker("p2", path.p2.x, path.p2.y);
  InteractiveMarker p3_marker = createControlMarker("p3", path.p3.x, path.p3.y);

  server->insert(p0_marker, boostControlMarkerFeedback);
  server->insert(p1_marker, boostControlMarkerFeedback);
  server->insert(p2_marker, boostControlMarkerFeedback);
  server->insert(p3_marker, boostControlMarkerFeedback);

  server->applyChanges();
}

MarkerArray Visuals::createPathVisual(const BezierSegment &s, size_t path_size)
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

void Visuals::updatePathVisual(MarkerArray *path_visual, const BezierSegment &s, size_t path_size)
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
    point_3d.z = height / 2.0;
    path_visual->markers[1].points[i] = point_3d;
  }
}

std::string Visuals::markerString(const std::string &name, double x, double y)
{
  std::stringstream ss;
  ss.precision(precision);
  ss << std::fixed << name << "("  << x << ", " << y << ")";
  return ss.str();
}
