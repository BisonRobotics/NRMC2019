#include <opencv2/opencv.hpp>

#include <occupancy_grid/arena_dimensions.h>
#include <occupancy_grid/point.h>

#ifndef OCCUPANCY_GRID_CIRCLE_H
#define OCCUPANCY_GRID_CIRCLE_H

namespace occupancy_grid
{
  class Circle
  {
    public:
      const Point p;
      const double r;

      Circle()                              : p(       0.0,        0.0), r(0.0)      {};
      Circle(Circle const &circle)          : p(circle.p.x, circle.p.y), r(circle.r) {};
      Circle(Circle const &&circle)         : p(circle.p.x, circle.p.y), r(circle.r) {};
      Circle(Point const &origin, double r) : p(  origin.x,   origin.y), r(r)        {};
      Circle(double x, double y, double r)  : p(         x,          y), r(r)        {};

      cv::Point2i imgTfP() const
      {
        return p.imgTf();
      }

      int imgTfR() const
      {
        return (int)(this->r / ArenaDimensions::resolution);
      }

  };
}



#endif //OCCUPANCY_GRID_CIRCLE_H
