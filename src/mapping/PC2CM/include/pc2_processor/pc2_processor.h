#ifndef PC2_PROCESSOR
#define PC2_PROCESSOR


#define _USE_MATH_DEFINES
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>

// using namespace cv;
/*========
pc2cmProcessor
class that takes incoming point cloud data and converts it to a costmap
operation is 2 phase:
1. Exposure: call addPoints() for incoming pc data for as long as desired
2. Compute: call computeOccupancyGrid() to crunch input'd data into costmap data
*/


class pc2cmProcessor
{
public:
    pc2cmProcessor(); // empty contructor

    pc2cmProcessor(double cell_width, double grid_width, double grid_length);   //constructor, should take cell_width, Grid_width, grid_length  (Width of a cell in the grid)

    bool addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg); // Adds points to internal point grid data

    bool addPoints(pcl::PointCloud<pcl::PointXYZ> cloud_msg);
    bool addPoint(pcl::PointXYZ point);

    double get_Height(int xindex, int yindex); // get a height from the map at the cordiates from the xindex, yindex (robot relative)
    void print_grid(void);

    void computeOccupancyGrid(nav_msgs::OccupancyGrid *costMap); //computes costmap from data by calling protected member functions correctly

    cv::Mat takeDoG(int kernel_size, double sigma1, double sigma2); //takes difference of gaussian of internal height grid

};

#endif