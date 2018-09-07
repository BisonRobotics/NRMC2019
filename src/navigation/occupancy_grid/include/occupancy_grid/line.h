#include <opencv2/opencv.hpp>

#include <arena.h>
#include <occupancy_grid/point.h>

#ifndef OCCUPANCY_GRID_LINE_H
#define OCCUPANCY_GRID_LINE_H

namespace occupancy_grid
{
  class Line
  {
    public:
      Line(Point &p0, Point &p1)
      {
        this->p0 = p0;
        this->p1 = p1;
      };

      Point &operator[](int i)
      {
        if (i == 0)
        {
          return this->p0;
        }
        else if (i == 1)
        {
          return this->p1;
        }
        else
        {
          throw std::out_of_range("[line.h] out of range 0:1");
        }
      }

      Point p0;
      Point p1;
  };
}



#endif //OCCUPANCY_GRID_LINE_H
