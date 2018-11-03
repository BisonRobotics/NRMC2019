#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pc2_processor/pc2_processor.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <boost/timer/timer.hpp>

using std::cout;
using std::endl;
using std::vector;
using ros::package::getPath;
using pcl::PointCloud;
using pcl::PointXYZ;
using boost::timer::cpu_timer;
using cv::Mat_;
using cv::filter2D;

int main(int argc, char** argv)
{
  const double cell_width = 0.25;
  const double map_width  = 4.0;
  const double map_height = 4.0;
  const int map_width_i = (int) (map_width / cell_width);
  const int map_height_i = (int) (map_height / cell_width);

  pc2cmProcessor processor(cell_width, map_width, map_height);
  processor.takeDoG(9, .8, .2);

  ros::init(argc, argv, "pc2cm_image_test");
  rosbag::Bag bag;
  bag.open(getPath("pc2cm") + "/bag/justpoints.bag", rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery("/camera/points"));

  cpu_timer timer;
  cout << "Entering processing loop" << endl;
  BOOST_FOREACH (rosbag::MessageInstance const message, view)
  {
    PointCloud<PointXYZ>::ConstPtr cloud = message.instantiate<PointCloud<PointXYZ>>();
    for (const PointXYZ &point : cloud->points)
    {
      if (!(point.x == 0.0 && point.y == 0.0 && point.z == 0.0))
      {
        processor.addPoint(point);
      }
    }
  }
  cout << "Processing Time:" << timer.format(2) << endl;
  processor.print_grid();
  Mat_<double> image(map_width_i, map_height_i, CV_64F);
  processor.copyMap(&image);
  imwrite("test.jpg", image);
}