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

int main( int argc, char** argv )
{
  using AD = ArenaDimensions;

  std::vector<Circle> rocks;
  rocks.emplace_back(3.0,  1.0, 0.3);
  rocks.emplace_back(4.0, -0.5, 0.3);
  rocks.emplace_back(2.5, -0.9, 0.3);
  Arena arena(rocks);

  Image obstacles_layer(AD::height_cm, AD::width_cm, Color::white());
  convert(&obstacles_layer, arena.inflated_obstacles);

  Image path_layer(AD::height_cm, AD::width_cm, Color::white());
  Point start(0.75, -0.75), control1(4.0, 1.0), control2(5.0, -0.4), finish(6.5, 0.8);
  Bezier curve(start, control1, control2, finish);
  draw(&path_layer, curve, Color::orange_red());
  std::cout << "Min radius of curvature: " << curve.min_radius(100) << std::endl;

  Image final(AD::height_cm, AD::width_cm);
  overlay(&final, path_layer, obstacles_layer);
  write(final);

  return 0;
}