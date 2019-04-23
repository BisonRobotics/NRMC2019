#ifndef COMPETITION_VISUALS_H
#define COMPETITION_VISUALS_H

#include <ros/ros.h>
#include <tf2/tr
#include <navigation_msgs/BezierSegment.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <occupancy_grid/bezier.h>

namespace competition
{
  using visualization_msgs::Marker;
  using visualization_msgs::MarkerArray;
  using visualization_msgs::InteractiveMarker;
  using visualization_msgs::InteractiveMarkerControl;
  using visualization_msgs::InteractiveMarkerFeedback;
  using interactive_markers::InteractiveMarkerServer;
  using navigation_msgs::BezierSegment;
  using occupancy_grid::Bezier;
  using geometry_msgs::Point;

  class Visuals
  {
  public:
    Visuals(ros::NodeHandle *nh);

    void update();
    void update(tf2::Transform transform);

    std::string markerString(const std::string &name, double x, double y);
    void controlMarkerFeedback(const InteractiveMarkerFeedback::ConstPtr &feedback);
    visualization_msgs::InteractiveMarker createControlMarker(std::string name, double x, double y);
    void createControlMarkers(InteractiveMarkerServer *server);
    MarkerArray createPathVisual(const BezierSegment &s, size_t path_size);
    void updatePathVisual(MarkerArray *path_visual, const BezierSegment &s, size_t path_size);

  private:
    boost::function<void (const InteractiveMarkerFeedback::ConstPtr &feedback)> boostControlMarkerFeedback;
    size_t size;
    uint precision;
    double height, width, text_size;
    MarkerArray path_visual;
    BezierSegment path;

    ros::NodeHandle *nh;
    InteractiveMarkerServer marker_server;
    ros::Publisher marker_pub;

  };
}

#endif //COMPETITION_VISUALS_H
