#include <math.h>
#include <exception>

#include <occupancy_grid/point.h>

#ifndef OCCUPANCY_GRID_BEZIER_H
#define OCCUPANCY_GRID_BEZIER_H


// See: https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves
namespace occupancy_grid
{
class Bezier
{
  private:
    using Point = occupancy_grid::Point;
    using PointRef = Point const &;

  public:
    const Point p0,p1,p2,p3;

    Bezier(Point const &p0, Point const &p1, Point const &p2, Point const &p3)
              : p0(p0), p1(p1), p2(p2), p3(p3) {};
    Bezier(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3)
              : p0(x0,y0), p1(x1,y1), p2(x2,y2), p3(x3,y3) {};

    Point operator()(double t) const
    {
      if (t < 0.0 || t > 1.0)
      {
        std::stringstream message;
        message << "[Bezier.h] out of range " << t << "!=[0:1]";
        throw std::out_of_range(message.str());
      }

      return std::pow(1-t,3)*p0 + 3*pow(1-t,2)*t*p1 + 3*(1-t)*pow(t,2)*p2 + pow(t,3)*p3;
    }
};
}



#endif //OCCUPANCY_GRID_BEZIER_H
