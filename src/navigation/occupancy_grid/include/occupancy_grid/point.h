#include <opencv2/opencv.hpp>

#include <arena.h>

#ifndef OCCUPANCY_GRID_POINT_H
#define OCCUPANCY_GRID_POINT_H

namespace occupancy_grid
{
  class Point : public cv::Point2d
  {
    public:
      Point() : Point_<double_t>(0.0, 0.0){};
      Point(double x, double y) : Point_<double_t>(x, y){};
      Point(Point &a) : Point_<double_t>(a){};

      cv::Point2i imgTf()
      {
        double x = (Arena::height      - this->x + 0.01) / Arena::resolution;
        double y = (Arena::width / 2.0 - this->y + 0.01) / Arena::resolution;
        return cv::Point2i((int)y, (int)x);
      }

      double &operator[](int i)
      {
        if (i == 0)
        {
          return this->x;
        }
        else if (i == 1)
        {
          return this->y;
        }
        else
        {
          throw std::out_of_range("[point.h] out of range 0:1");
        }
      }

  };
}



#endif //OCCUPANCY_GRID_POINT_H
