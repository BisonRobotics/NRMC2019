#include <path_planner/path_planner.h>

using namespace path_planner;

using occupancy_grid::OrderByLowestCost;

PathPlanner::PathPlanner(double radius_step_size, double max_radius,
                         double min_curve_radius, double max_obstacle_cost,
                         int    arc_steps,        int    cost_steps)
    : radius_step_size(radius_step_size),
      max_radius(max_radius),
      min_curve_radius(min_curve_radius),
      max_obstacle_cost(max_obstacle_cost),
      arc_steps(arc_steps),
      cost_steps(cost_steps)
{

}

std::vector<Point> PathPlanner::controlPoints(const Point &a)
{
  int r_steps   = (int)(max_radius / radius_step_size); // Number of radial steps
  double r_step = max_radius / r_steps;    // Radial step size
  double arc_step = 2.0 * M_PI / arc_steps;

  std::vector<Point> points;
  for (int i = 1; i <= r_steps; i++)
  {
    double r = r_step*i;
    for (int j = 1; j <= arc_steps; j++)
    {
      points.emplace_back(r*cos(arc_step*j) + a.x, r*sin(arc_step*j) + a.y);
    }
  }
  return points;
}

std::vector<Point> PathPlanner::controlPoints(const Line &a)
{
  int  r_steps = (int)(max_radius / radius_step_size); // Number of radial steps
  Point r_step = radius_step_size * ((a.p1 - a.p0) / a.magnitude());

  std::vector<Point> points;
  for (int i = 1; i <= r_steps; i++)
  {
    points.emplace_back(i*r_step + a.p0);
  }
  return points;
}

Bezier PathPlanner::findSegment(const OccupancyGrid &obstacles,
                                const std::vector<Point> &p0, const std::vector<Point> &p1,
                                const std::vector<Point> &p2, const std::vector<Point> &p3)
{
  std::cout << "Search size: " << p0.size() * p1.size() * p2.size() * p3.size() << std::endl;

  std::priority_queue<Bezier, std::vector<Bezier>, OrderByLowestCost> p_queue;

  for (int i = 0; i < p0.size(); i++)
  {
    for (int j = 0; j < p1.size(); j++)
    {
      for (int k = 0; k < p2.size(); k++)
      {
        for (int l = 0; l < p3.size(); l++)
        {
          Bezier current_curve(p0[i], p1[j], p2[k], p3[l]);
          current_curve.updateCost(obstacles, cost_steps);

          double current_radius = current_curve.min_radius(20);
          if (current_radius > min_curve_radius && current_curve.max_cost < max_obstacle_cost)
          {
            p_queue.push(current_curve);
          }
        }
      }
    }
  }
  if (p_queue.empty())
  {
    throw std::runtime_error("Unable to plan a path"); // TODO use a custom exception
  }
  return p_queue.top();
}

Bezier PathPlanner::findSegment(OccupancyGrid const &obstacles, Point const &start, Point const &finish)
{
  std::vector<Point> p0,p3;
  p0.emplace_back(start);
  std::vector<Point> p1 = controlPoints(start);
  std::vector<Point> p2 = controlPoints(finish);
  p3.emplace_back(finish);

  return findSegment(obstacles, p0, p1, p2, p3);
}

Bezier PathPlanner::findSegment(OccupancyGrid const &obstacles, Line const &start, Point const &finish)
{
  std::vector<Point> p0,p3;
  p0.emplace_back(start.p0);
  std::vector<Point> p1 = controlPoints(start);
  std::vector<Point> p2 = controlPoints(finish);
  p3.emplace_back(finish);

  return findSegment(obstacles, p0, p1, p2, p3);
}

Bezier PathPlanner::findSegment(OccupancyGrid const &obstacles, Point const &start, Line const &finish)
{
  std::vector<Point> p0,p3;
  p0.emplace_back(start);
  std::vector<Point> p1 = controlPoints(start);
  std::vector<Point> p2 = controlPoints(finish);
  p3.emplace_back(finish.p0);

  return findSegment(obstacles, p0, p1, p2, p3);
}

Bezier PathPlanner::findSegment(OccupancyGrid const &obstacles, Line const &start, Line const &finish)
{
  std::vector<Point> p0,p3;
  p0.emplace_back(start.p0);
  std::vector<Point> p1 = controlPoints(start);
  std::vector<Point> p2 = controlPoints(finish);
  p3.emplace_back(finish.p0);

  return findSegment(obstacles, p0, p1, p2, p3);
}
