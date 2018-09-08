#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>

#ifndef OCCUPANCY_GRID_DISPLAY_H
#define OCCUPANCY_GRID_DISPLAY_H

namespace occupancy_grid
{
static void show(cv::Mat const &mat)
{
  namedWindow( "Display", cv::WINDOW_NORMAL );
  imshow( "Display", mat);
  cv::waitKey(0);
}

static void write(cv::Mat_<double> const &mat)
{
  cv::Mat_<double> image;
  mat.copyTo(image);
  image = 255.0 - 255.0 * image;
  imwrite("occupancy_grid.png", image);
}

static void write(cv::Mat3b const &mat)
{
  cv::Mat3b image;
  mat.copyTo(image);
  imwrite("occupancy_grid.png", image);
}

// Overlay a onto b
static void overlay(cv::Mat3b *out, cv::Mat3b const &a, cv::Mat3b const &b)
{
  for (int i = 0; i < a.rows; i++)
  {
    for (int j = 0; j < a.cols; j++)
    {
      cv::Scalar value = a(i,j);
      if (value != cv::viz::Color::white())
      {
        (*out)(i,j) = a(i,j);
      }
      else
      {
        (*out)(i,j) = b(i,j);
      }
    }
  }
}

}

#endif //OCCUPANCY_GRID_DISPLAY_H
