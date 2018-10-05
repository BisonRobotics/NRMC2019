#ifndef PC2_PROCESSOR
#define PC2_PROCESSOR


#define _USE_MATH_DEFINES
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pcl/point_types.h>
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
/*========
pc2cmProcessor
class that takes incoming point cloud data and converts it to a costmap
operation is 2 phase:
1. Exposure: call addPoints() for incoming pc data for as long as desired
2. Compute: call computeCostmap() to crunch input'd data into costmap data
*/


class pc2cmProcessor
{
public:
    pc2cmProcessor(); // empty contructor

    pc2cmProcessor(double cell_width, double grid_width, double grid_length);   //constructor, should take cell_width, Grid_width, grid_length  (Width of a cell in the grid)


    bool addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg); // Adds points to internal point grid data
    bool addPoints(pcl::PointCloud<pcl::PointXYZ> cloud_msg);

    double get_Height(int xindex, int yindex); // get a height from the map at the cordiates from the xindex, yindex (robot relative)
    void print_grid(void);

    costmap_2d::Costmap2DROS computeCostmap(); //computes costmap from data by calling protected member functions correctly

    cv::Mat takeDoG(int kernel_size, double sigma1, double sigma2);
    //takes difference of gaussian of internal height grid



// private:

/*  float heights[width][height]
    //internal grid of heights
*/
/*  float costmap[width][height]
    //internal costmap representation computed with takeDoG()
*/


};

#endif