#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/viz/types.hpp>

#include <occupancy_grid/arena.h>
#include <occupancy_grid/display.h>

using namespace occupancy_grid;
using namespace cv::viz;

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
  Bezier a(1.0,0.5, 2.0,1.5, 3.5,-1.0, 4.0,-0.5);
  draw(&path_layer, a, Color::orange_red());

  Image final(AD::height_cm, AD::width_cm);
  overlay(&final, path_layer, obstacles_layer);
  write(final);

  return 0;
}