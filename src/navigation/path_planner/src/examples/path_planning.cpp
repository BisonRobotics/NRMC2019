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

int main( int argc, char** argv )
{
  using AD = ArenaDimensions;

  std::vector<Circle> rocks;
  rocks.emplace_back(3.0,  1.0, 0.3);
  rocks.emplace_back(4.0,  1.0, 0.3);
  rocks.emplace_back(2.5, -0.9, 0.3);
  Arena arena(rocks);

  Image obstacle_layer(AD::height_cm, AD::width_cm, Color::white());
  convert(&obstacle_layer, arena.inflated_obstacles);

  Image path_layer(AD::height_cm, AD::width_cm, Color::white());
  Point start(0.75, -0.75);
  Point finish(6.5, 0.8);

  try
  {
    cpu_timer timer;
    PathPlanner planner;
    Bezier best_curve = planner.findSegment(arena.inflated_obstacles, start, finish);
    std::cout << "Time:" << timer.format(2) << std::endl;
    std::cout << "Path cost: " << best_curve.path_cost << ", Max cost: " << best_curve.max_cost << std::endl;

    draw(&path_layer, best_curve, Color::orange_red());
    std::cout << "Min radius of curvature: " << best_curve.min_radius(20) << std::endl;
  }
  catch (std::runtime_error &e)
  {
    std::cerr << e.what() << std::endl;
  }

  Image final(AD::height_cm, AD::width_cm);
  overlay(&final, path_layer, obstacle_layer);
  write(final);

  return 0;
}