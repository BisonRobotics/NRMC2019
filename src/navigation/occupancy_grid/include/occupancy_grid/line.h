#include <opencv2/opencv.hpp>

#include <occupancy_grid/arena_dimensions.h>
#include <occupancy_grid/point.h>

#ifndef OCCUPANCY_GRID_LINE_H
#define OCCUPANCY_GRID_LINE_H

namespace occupancy_grid
{
  class Line
  {
    public:
      const Point p0, p1;

      Line(Point p0, Point p1) : p0(p0.x, p0.y), p1(p1.x, p1.y) {};
      Line(Line const &line)   : p0(line.p0),    p1(line.p1)    {};
      Line(Line const &&line)  : p0(line.p0),    p1(line.p1)    {};
      Line(double x0, double y0, double x1, double y1) : p0(x0,y0), p1(x1,y1) {};

      const Point &operator[](int i) const
      {
        if (i == 0)
        {
          return p0;
        }
        else if (i == 1)
        {
          return p1;
        }
        else
        {
          throw std::out_of_range("[line.h] out of range 0:1");
        }
      }
  };
}



#endif //OCCUPANCY_GRID_LINE_H