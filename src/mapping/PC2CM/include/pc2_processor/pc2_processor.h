#ifndef PC2_PROCESSOR
#define PC2_PROCESSOR


#define _USE_MATH_DEFINES
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

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
    pc2cmProcessor(double cell_width, double grid_width, double grid_length);   //constructor, should take cell_width, Grid_width, grid_length  (Width of a cell in the grid)

    bool getOne();

    bool addPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg); // Adds points to internal point grid data

/*  costmap computeCostmap();
    //computes costmap from data by calling protected member functions correctly
*/
//protected:    //these members should be treated as protected,
                //but for unit testing are actually public

//    void averagePoints();//computes heights of grid elements by averaging all added points in that cell's range

/*  void takeDoG(double radius_1, double radius_2);
    //takes difference of gaussian of internal height grid
*/


private:

/*  float heights[width][height]
    //internal grid of heights
*/
/*  float costmap[width][height]
    //internal costmap representation computed with takeDoG()
*/


};

#endif