#include <opencv2/opencv.hpp>

#ifndef OCCUPANCY_GRID_IMAGE_H
#define OCCUPANCY_GRID_IMAGE_H

namespace occupancy_grid
{

class Image : public cv::Mat_<cv::Vec3b>
{
  public:
    Image(int rows, int cols) : cv::Mat_<cv::Vec3b>(rows, cols, 0.0)
    {};

    Image(int rows, int cols, cv::Vec3b value) : cv::Mat_<cv::Vec3b>(rows, cols, value)
    {};
};

}

#endif //OCCUPANCY_GRID_IMAGE_H
