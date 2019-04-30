#ifndef COMPETITION_WAYPOINT_WaypointVisuals_H
#define COMPETITION_WAYPOINT_WaypointVisuals_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <waypoint_control/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>


namespace competition
{
  using waypoint_control::Waypoint;
  using Waypoints = std::vector<waypoint_control::Waypoint>;

  using visualization_msgs::Marker;
  using visualization_msgs::MarkerArray;
  using visualization_msgs::InteractiveMarker;
  using visualization_msgs::InteractiveMarkerControl;
  using visualization_msgs::InteractiveMarkerFeedback;
  using interactive_markers::InteractiveMarkerServer;
  using geometry_msgs::Point;

  class WaypointVisuals
  {
  public:
    WaypointVisuals(ros::NodeHandle *nh);

    void update(tf2::Transform robot);
    void updateWaypoints(const Waypoints &s);
    void addWaypoint(const geometry_msgs::PointStampedConstPtr &point);
    void setReverse(bool reverse);
    void clearWaypoints();
    Waypoints getWaypoints();
    void followRobot(bool follow);
    void dynamicWaypoints(bool dynamic_waypoints);

    std::string markerString(const std::string &name, double x, double y);
    std::string markerString(const std::string &name, double x, double y, double theta);
    void controlMarkerFeedback(const InteractiveMarkerFeedback::ConstPtr &feedback);
    void addControlMarker(std::string name, double x, double y);
    void updateWaypointVisuals(const Waypoints &s, const tf2::Transform &robot_transform);

  private:
    boost::function<void (const InteractiveMarkerFeedback::ConstPtr &feedback)> boostControlMarkerFeedback;
    uint seq;
    size_t size;
    uint precision;
    double height, width, text_size;
    MarkerArray waypoint_visuals;
    Waypoints waypoints;
    bool dynamic_waypoints, follow_robot;
    tf2::Transform robot;
    bool reverse;

    ros::NodeHandle *nh;
    InteractiveMarkerServer marker_server;
    ros::Publisher waypoints_pub;
    ros::Subscriber add_waypoint_sub;

  };
}

#endif //COMPETITION_WAYPOINT_WaypointVisuals_H
