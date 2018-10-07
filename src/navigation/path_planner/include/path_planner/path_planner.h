#include <queue>

#include <occupancy_grid/point.h>
#include <occupancy_grid/line.h>
#include <occupancy_grid/bezier.h>
#include <occupancy_grid/occupancy_grid.h>

#ifndef PATH_PLANNER_PATH_PLANNER_H
#define PATH_PLANNER_PATH_PLANNER_H

namespace path_planner
{

using occupancy_grid::Point;
using occupancy_grid::OccupancyGrid;
using occupancy_grid::Bezier;
using occupancy_grid::Line;

class PathPlanner
{
  public:

    double radius_step_size, max_radius, min_curve_radius, max_obstacle_cost;
    int arc_steps, cost_steps;

    PathPlanner(double radius_step_size  = 0.5,
                double max_radius        = 6.0,
                double min_curve_radius  = 0.8,
                double max_obstacle_cost = 0.8,
                int    arc_steps         = 15,
                int    cost_steps        = 20);

    std::vector<Point> controlPoints(Point const &a);
    std::vector<Point> controlPoints(Line  const &a);

    Bezier findSegment(OccupancyGrid const &obstacles,
                       std::vector<Point> const &p0, std::vector<Point> const &p1,
                       std::vector<Point> const &p2, std::vector<Point> const &p3);
    Bezier findSegment(OccupancyGrid const &obstacles, Point const &start, Point const &finish);
    Bezier findSegment(OccupancyGrid const &obstacles, Line  const &start, Point const &finish);
    Bezier findSegment(OccupancyGrid const &obstacles, Point const &start, Line  const &finish);
    Bezier findSegment(OccupancyGrid const &obstacles, Line  const &start, Line  const &finish);
};

}

#endif //PATH_PLANNER_PATH_PLANNER_H
