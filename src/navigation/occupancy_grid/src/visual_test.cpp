#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <boost/timer/timer.hpp>

#include <occupancy_grid/arena.h>
#include <occupancy_grid/display.h>

using namespace occupancy_grid;
using namespace cv::viz;
using boost::timer::cpu_timer;

typedef std::vector<Point> Points;

/*
 * a         - Point of intersection
 * step_size - Approximate step size
 */
Points control_points(Point a, double step_size, double max_radius)
{
  int r_steps   = (int)(max_radius / step_size); // Number of radial steps
  double r_step = max_radius / r_steps;    // Radial step size

  std::vector<Point> points;
  for (int i = 1; i <= r_steps; i++)
  {
    double r = r_step*i;
    int    c_steps = (int)(2.0*M_PI*r/step_size); // Number of circumference steps
    double c_step  = 2.0*M_PI / c_steps;
    for (int j = 1; j <= c_steps; j++)
    {
      points.emplace_back(r*cos(c_step*j) + a.x, r*sin(c_step*j) + a.y);
    }
  }
  return points;
}

Bezier find_path(Point start, Point finish)
{
  Points start_cp  = control_points(start, 0.5, 3.0);
  Points finish_cp = control_points(finish, 0.5, 3.0);

  Bezier best_curve(start, start, finish, finish);
  double highest_radius = 0.0;
  for (int i = 0; i < start_cp.size(); i++)
  {
    for (int j = 0; j < finish_cp.size(); j++)
    {
      Bezier current_curve(start, start_cp[i], finish_cp[j], finish);
      double current_radius = current_curve.min_radius();
      if (highest_radius < current_radius)
      {
        best_curve = current_curve;
        highest_radius = current_radius;
      }
    }
  }
  return best_curve;
}

int main( int argc, char** argv )
{
  using AD = ArenaDimensions;

  std::vector<Circle> rocks;
  rocks.emplace_back(5.0, 0.0, 0.3);
  rocks.emplace_back(2.3, -1.0, 0.3);
  rocks.emplace_back(1.8,  0.5, 0.3);
  Arena arena(rocks);

  Image obstacles_layer(AD::height_cm, AD::width_cm, Color::white());
  convert(&obstacles_layer, arena.inflated_obstacles);

  Image path_layer(AD::height_cm, AD::width_cm, Color::white());
  Point start(1.0, -0.5);
  Point finish(6.0, 0.5);

  cpu_timer timer;
  Bezier best_curve = find_path(start, finish);
  std::cout << "Time:" << timer.format(2) << std::endl;

 // Bezier best_curve(start, Point(1.5,0.4), Point(3.5,-0.4), finish);
  draw(&path_layer, best_curve, Color::orange_red());
  std::cout << "Min radius of curvature: " << best_curve.min_radius(100) << std::endl;

  //std::vector<Point> points = control_points(start, 0.5, 3.0);


  Image final(AD::height_cm, AD::width_cm);
  overlay(&final, path_layer, obstacles_layer);
  write(final);

  return 0;
}