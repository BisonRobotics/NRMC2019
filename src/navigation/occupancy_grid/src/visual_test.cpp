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

  //Image obstacles(AD::height_cm, AD::width_cm);
  cv::Mat image(AD::width_cm, AD::height_cm, CV_8UC1);
  image = 255 - 255 * arena.inflated_obstacles;
  image.convertTo(image, CV_8UC1);
  cvtColor(image, image, cv::COLOR_GRAY2BGR);
  //write((cv::Mat3b)image);
  //std::cout << arena.walls(AD::height_cm - 1, AD::width_cm / 2 - 1) << std::endl;
  //write(arena.inflated_obstacles);

  Image grid(AD::height_cm, AD::width_cm, Color::white());
  Bezier a(1.0,0.5,1.5,1.0, 3.5,-1.0,4.0,-0.5);
  draw(&grid, a, Color::orange_red());

  Image final(AD::height_cm, AD::width_cm);
  overlay(&final, grid, image);
  write(final);

  //blurred.write();
  // Wait for a keystroke in the window
  return 0;
}