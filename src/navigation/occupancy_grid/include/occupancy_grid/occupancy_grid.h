#include <opencv2/opencv.hpp>
#include <math.h>

#include <occupancy_grid/point.h>
#include <occupancy_grid/line.h>
#include <occupancy_grid/circle.h>
#include <occupancy_grid/bezier.h>

#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_OCCUPANCY_GRID_H

namespace occupancy_grid
{

class OccupancyGrid : public cv::Mat_<double>
{
  public:
    OccupancyGrid(int rows, int cols) : cv::Mat_<double>(rows, cols, 0.0) {};
    OccupancyGrid(int rows, int cols, double value) : cv::Mat_<double>(rows, cols, value) {};

    static void max(OccupancyGrid *out, OccupancyGrid const &a, OccupancyGrid const &b);
    static void inflate(OccupancyGrid *out, OccupancyGrid const &in,
                        double cutoff=0.7, int kernel_size=100, int passes=2);
};



}

#endif //OCCUPANCY_GRID_OCCUPANCY_GRID_H
