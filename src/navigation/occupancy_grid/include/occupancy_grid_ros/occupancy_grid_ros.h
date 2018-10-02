#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>

#include <occupancy_grid/arena_dimensions.h>
#include <occupancy_grid/display.h>

#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_ROS_H
#define OCCUPANCY_GRID_OCCUPANCY_GRID_ROS_H

namespace occupancy_grid
{

void updateHeader(nav_msgs::OccupancyGrid *b, unsigned int seq, ros::Time stamp, ros::Time load_time)
{
  using AD = ArenaDimensions;

  b->header.seq = seq;
  b->header.stamp = stamp;
  b->header.frame_id = "map";
  b->info.map_load_time = load_time;
  b->info.resolution = (float)AD::resolution;
  b->info.width = AD::width_cm;
  b->info.height = AD::height_cm;
  b->info.origin.position.x = AD::height;
  b->info.origin.position.y = -AD::width / 2;

  tf2::Quaternion rotation(tf2::Vector3(0.0, 0.0, 1.0), M_PI_2);
  tf2::convert(rotation, b->info.origin.orientation);

}

void convert(OccupancyGrid const &a, nav_msgs::OccupancyGrid *b)
{
  cv::Mat_<double> image_f(ArenaDimensions::width_cm, ArenaDimensions::height_cm, CV_64FC1);
  cv::Mat_<int8_t> image_i(ArenaDimensions::width_cm, ArenaDimensions::height_cm, CV_8SC1);
  image_f = 100.0 * a;
  image_f.convertTo(image_i, CV_8SC1);
  cv::flip(image_i, image_i, 1);
  image_i = image_i.reshape(1,1);
  b->data.insert(b->data.begin(), image_i.begin(), image_i.end());
}

void convert(nav_msgs::OccupancyGrid const &a, OccupancyGrid *b)
{
  cv::Mat_<double> image_f(ArenaDimensions::height_cm, ArenaDimensions::width_cm, CV_64FC1);
  cv::Mat_<int8_t> image_i(ArenaDimensions::height_cm, ArenaDimensions::width_cm, (int8_t*)a.data.data());
  image_i.convertTo(image_f, CV_64FC1);
  image_f = image_f / 100.0;
  image_f.copyTo(*b);
}

void updateHeader(nav_msgs::Path *b, unsigned int seq, ros::Time stamp)
{
  b->header.seq = seq;
  b->header.stamp = stamp;
  b->header.frame_id = "map";

}

void convert(Bezier const &a, nav_msgs::Path *b, int steps)
{
  double step = 1.0 / steps;
  for (int i = 0; i < steps; i++)
  {
    Point point = a(0, step*i);
    Point d_point = a(1, step*i);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion rotation(tf2::Vector3(0.0, 0.0, 1.0), atan2(d_point.y, d_point.x));
    tf2::convert(rotation, pose.pose.orientation);
    b->poses.emplace_back(pose);
  }
}

}

#endif //OCCUPANCY_GRID_OCCUPANCY_GRID_ROS_H
