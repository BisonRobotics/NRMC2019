#include <opencv2/opencv.hpp>

#include <arena.h>
#include <occupancy_grid/point.h>

#ifndef OCCUPANCY_GRID_CIRCLE_H
#define OCCUPANCY_GRID_CIRCLE_H

namespace occupancy_grid
{
  class Circle
  {
    public:
      Circle(Point &p, double r)
      {
        this->p = p;
        this->r = r;
      }

      cv::Point2i imgTfP()
      {
        return p.imgTf();
      }

      int imgTfR()
      {
        return (int)(this->r / Arena::resolution);
      }

      Point p;
      double r;
  };
}



#endif //OCCUPANCY_GRID_CIRCLE_H
