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
  public:
    Point p0,p1,p2,p3;
    double path_cost, max_cost;

    Bezier() : p0(0,0), p1(0,0), p2(0,0), p3(0,0), path_cost(-1.0), max_cost(-1.0) {};
    Bezier(Point const &p0, Point const &p1, Point const &p2, Point const &p3)
              : p0(p0), p1(p1), p2(p2), p3(p3), path_cost(-1.0), max_cost(-1.0) {};
    Bezier(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3)
              : p0(x0,y0), p1(x1,y1), p2(x2,y2), p3(x3,y3), path_cost(-1.0), max_cost(-1.0) {};
    Bezier(Bezier const &a) : p0(a.p0), p1(a.p1), p2(a.p2), p3(a.p3), path_cost(a.path_cost), max_cost(a.max_cost) {};

    Point operator()(int order, double t) const
    {
      if (t < 0.0 || t > 1.0)
      {
        std::stringstream message;
        message << "[Bezier.h] t = " << t << " must be in range [0:1]";
        throw std::out_of_range(message.str());
      }
      if (order == 0)
      {
        return pow(1-t,3)*p0 + 3*pow(1-t,2)*t*p1 + 3*(1-t)*pow(t,2)*p2 + pow(t,3)*p3;
      }
      else if (order == 1)
      {
        return 3*pow(1-t,2)*(p1-p0) + 6*(1-t)*t*(p2-p1) + 3*pow(t,2)*(p3-p2);
      }
      else if (order == 2)
      {
        return 6*(1-t)*(p2 - 2*p1 + p0) + 6*t*(p3 - 2*p2 + p1);
      }
      else
      {
        std::stringstream message;
        message << "[Bezier.h] order = " << order << " must be in the set {0,1,2}";
        throw std::out_of_range(message.str());

      }
    }

    // Curvature
    double curvature(double t) const
    {
      Point db  = (*this)(1,t);
      Point d2b = (*this)(2,t);
      double denominator = pow((pow(db.x,2) + pow(db.y,2)), 1.5);
      if (denominator == 0.0)
      {
        return std::numeric_limits<double>::max();
      }
      else
      {
        return std::abs(db.x*d2b.y - db.y*d2b.x) / denominator;
      }
    }

    // Radius of curvature
    double radius(double t) const
    {
      Point db  = (*this)(1,t);
      Point d2b = (*this)(2,t);
      double denominator = std::abs(db.x*d2b.y - db.y*d2b.x);
      if (denominator == 0.0)
      {
        return std::numeric_limits<double>::max();
      }
      else
      {
        return pow((pow(db.x,2) + pow(db.y,2)), 1.5) / denominator;
      }
    }

    // Minimum radius of curvature
    double min_radius(int steps=10) const
    {
      double step = 1.0 / steps;
      double current_min = std::numeric_limits<double>::max();
      for (double t = 0; t < 1.0; t += step)
      {
        double current_radius = this->radius(t);
        current_min = std::min(current_min, current_radius);
      }
      return current_min;
    }

    void updateCost(const cv::Mat_<double> &obstacles, int cost_steps)
    {
      this->path_cost = 0.0;
      this->max_cost  = 0.0;
      double step_size = 1.0 / cost_steps;
      for (int i = 0; i < cost_steps; i++)
      {
        Point pointf = (*this)(0, step_size*i);
        cv::Point2i point = pointf.imgTf();
        double current_value = obstacles(point.y, point.x);

        this->path_cost = this->path_cost + current_value;
        this->max_cost = std::max(current_value, this->max_cost);
      }

    }
};

struct OrderByLowestCost
{
  bool operator() (Bezier const &a, Bezier const &b) { return a.path_cost > b.path_cost; }
};

}

#endif //OCCUPANCY_GRID_BEZIER_H
