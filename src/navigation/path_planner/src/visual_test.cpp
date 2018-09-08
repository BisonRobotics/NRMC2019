#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <occupancy_grid/arena_dimensions.h>
#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/point.h>
#include <occupancy_grid/arena.h>

#include <occupancy_grid/bezier.h>

using namespace occupancy_grid;

int main( int argc, char** argv )
{
  using AD = ArenaDimensions;

  std::vector<Circle> rocks;
  rocks.emplace_back(5.0, 0.0, 0.3);
  rocks.emplace_back(2.3, -1.0, 0.3);
  rocks.emplace_back(1.8,  0.5, 0.3);
  Arena arena(rocks);
  //std::cout << arena.walls(AD::height_cm - 1, AD::width_cm / 2 - 1) << std::endl;
  arena.inflated_obstacles.write();

  //blurred.write();
  // Wait for a keystroke in the window
  return 0;
}