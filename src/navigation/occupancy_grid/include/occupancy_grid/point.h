#include <opencv2/opencv.hpp>

#include <occupancy_grid/arena_dimensions.h>

#ifndef OCCUPANCY_GRID_POINT_H
#define OCCUPANCY_GRID_POINT_H

namespace occupancy_grid
{
  class Point
  {
    public:
      const double x, y;

      Point()                   : x(0.0), y(0.0) {};
      Point(double x, double y) : x(x),   y(y)   {};
      Point(Point const &p)     : x(p.x), y(p.y) {};
      Point(Point const &&p)    : x(p.x), y(p.y) {};

      cv::Point2i imgTf() const
      {
        double x = round((ArenaDimensions::height        - 0.01 - this->x) / ArenaDimensions::resolution);
        double y = round((ArenaDimensions::width / 2.0   - 0.01 - this->y) / ArenaDimensions::resolution);
        return cv::Point2i((int)y, (int)x);
      }

      const double &operator[](int i) const
      {
        if (i == 0)
        {
          return x;
        }
        else if (i == 1)
        {
          return y;
        }
        else
        {
          throw std::out_of_range("[point.h] out of range 0:1");
        }
      }
  };
}



#endif //OCCUPANCY_GRID_POINT_H
