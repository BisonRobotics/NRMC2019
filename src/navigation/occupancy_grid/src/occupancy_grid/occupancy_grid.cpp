#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/arena_dimensions.h>

using namespace occupancy_grid;

void OccupancyGrid::max(OccupancyGrid *out, OccupancyGrid const &a, OccupancyGrid const &b)
{
  for (int i = 0; i < a.rows; i++)
  {
    for (int j = 0; j < a.cols; j++)
    {
      (*out)(i,j) = std::max(a(i,j), b(i,j));
    }
  }
}

void OccupancyGrid::inflate(OccupancyGrid *out, OccupancyGrid const &in,
                            double cutoff, int kernel_size, int passes)
{
  // Averaging filter
  cv::blur(in, (*out), cv::Size(kernel_size, kernel_size));
  for (int i = 0; i < passes; i++)
  {
    cv::blur((*out), (*out), cv::Size(kernel_size, kernel_size));
  }

  // Normalize and threshold
  double min, max;
  cv::minMaxLoc((*out), &min, &max);
  for (int i = 0; i < out->rows; i++)
  {
    for (int j = 0; j < out->cols; j++)
    {
      (*out)(i,j) = (*out)(i,j) / max;
      if ((*out)(i,j) > cutoff)
      {
        (*out)(i,j) = 1.0;
      }
      else
      {
        (*out)(i,j) = (*out)(i,j) / cutoff;
      }
    }
  }
}

