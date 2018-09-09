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

}

}

#endif //OCCUPANCY_GRID_OCCUPANCY_GRID_ROS_H