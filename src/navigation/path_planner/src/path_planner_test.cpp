#include <iostream>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>
#include <boost/timer/timer.hpp>

#include <occupancy_grid/arena.h>
#include <occupancy_grid/display.h>
#include <path_planner/path_planner.h>

using namespace path_planner;
using namespace occupancy_grid;
using namespace cv::viz;
using boost::timer::cpu_timer;

/*Bezier findSegment(OccupancyGrid const &obstacles, Line const &start, Point const &finish)
{
  const double min_radius = 0.8;

  Point p0(start.p0);
  Point p3(finish);

  Points p1 = controlPoints(start, 1.0, 6.0, 15);
  Points p2 = controlPoints(finish, 1.0, 6.0, 15);
  std::priority_queue<Bezier, std::vector<Bezier>, OrderByLowestCost> p_queue;

  for (int i = 0; i < p1.size(); i++)
  {
    for (int j = 0; j < p2.size(); j++)
    {
      Bezier current_curve(p0, p1[i], p2[j], p3);
      updateCost(&current_curve, obstacles, 20);

      double current_radius = current_curve.min_radius();
      if (current_radius > min_radius && current_curve.max_cost < 0.80)
      {
        p_queue.push(current_curve);
      }
    }
  }
  return p_queue.top();
}*/

int main( int argc, char** argv )
{
  using AD = ArenaDimensions;

  std::vector<Circle> rocks;
  rocks.emplace_back(3.0,  1.0, 0.3);
  rocks.emplace_back(4.0, -0.5, 0.3);
  rocks.emplace_back(2.5, -0.9, 0.3);
  Arena arena(rocks);

  Image obstacle_layer(AD::height_cm, AD::width_cm, Color::white());
  convert(&obstacle_layer, arena.inflated_obstacles);

  Image path_layer(AD::height_cm, AD::width_cm, Color::white());
  Point start(0.75, -0.75);
  Point finish(6.5, 0.8);

  /* Path planning options */
  try
  {
    cpu_timer timer;

    PathPlanner planner;
    //Bezier best_curve(start, Point(4.0, 0.5), Point(6.5, -0.4), finish);
    //Bezier best_curve = planner.findSegment(arena.inflated_obstacles, start, finish);
    //Bezier best_curve = planner.findSegment(arena.inflated_obstacles, Line(start, Point(0.8, -0.8)), finish);
    //Bezier best_curve = planner.findSegment(arena.inflated_obstacles, finish, Line(0.01, 0.0, 1.0, 0.0));
    Bezier best_curve = planner.findSegment(arena.inflated_obstacles, Line(finish, Point(5.5, 0.95)),
                                            Line(0.01, 0.0, 1.0, 0.0));

    std::cout << "Time:" << timer.format(2) << std::endl;
    std::cout << "Path cost: " << best_curve.path_cost << ", Max cost: " << best_curve.max_cost << std::endl;

    draw(&path_layer, best_curve, Color::orange_red());
    std::cout << "Min radius of curvature: " << best_curve.min_radius(20) << std::endl;
  }
  catch (std::runtime_error &e)
  {
    std::cerr << e.what() << std::endl;
  }


  //std::vector<Point> points = control_points(start, 0.5, 3.0);


  Image final(AD::height_cm, AD::width_cm);
  overlay(&final, path_layer, obstacle_layer);
  write(final);

  return 0;
}