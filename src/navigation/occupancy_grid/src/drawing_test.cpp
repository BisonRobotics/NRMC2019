#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <arena.h>
#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/point.h>

using namespace occupancy_grid;

int main( int argc, char** argv )
{
  using AD = Arena;

  OccupancyGrid arena = OccupancyGrid(AD::height_cm, AD::width_cm);
  cv::Scalar fill = cv::Scalar((int8_t)100);
  Point start(5.0, 0.0);
  Point finish(2.0, 1.0);
  Circle circle(start, 1.0);
  Line path(start, finish);
  arena.draw(start);
  arena.draw(finish);
  arena.draw(path);
  arena.draw(circle);

  OccupancyGrid blurred(AD::height_cm, AD::width_cm);
  OccupancyGrid::inflate(arena, blurred, 0.8);
  //arena.write();
  blurred.write();
                                   // Wait for a keystroke in the window
  return 0;
}