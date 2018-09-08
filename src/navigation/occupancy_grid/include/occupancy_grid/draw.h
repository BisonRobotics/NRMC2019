#include <type_traits>

#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/image.h>

#ifndef OCCUPANCY_GRID_DRAW_H
#define OCCUPANCY_GRID_DRAW_H

namespace occupancy_grid
{

static void draw(cv::Mat *mat, Point const &a, cv::Scalar fill)
{
  cv::drawMarker((*mat), a.imgTf(), fill, cv::MARKER_TILTED_CROSS, 10, 4);
}

static void draw(cv::Mat *mat, Line const &a, cv::Scalar fill, int thickness = 1)
{
  cv::line((*mat), a.p0.imgTf(), a.p1.imgTf(), fill, thickness);
}

static void draw(cv::Mat *mat, Circle const &a, cv::Scalar fill)
{
  cv::circle((*mat), a.imgTfP(), a.imgTfR(), fill, -1);
}

static void draw(cv::Mat *mat, Bezier const &a, cv::Scalar fill)
{
  using cv::viz::Color;
  std::cout << mat->type() << " " << CV_8UC3 << std::endl;
  for (double i = 0; i < 0.99; i = i + 0.01)
  {
    cv::line((*mat), a(i).imgTf(), a(i + 0.01).imgTf(), Color::green(), 2);
  }
  if (mat->type() == CV_8UC3)
  {
    cv::line((*mat), a.p0.imgTf(), a.p1.imgTf(), Color::bluberry(), 2);
    cv::line((*mat), a.p2.imgTf(), a.p3.imgTf(), Color::bluberry(), 2);
    draw(mat, a.p0, Color::red());
    draw(mat, a.p1, Color::red());
    draw(mat, a.p2, Color::red());
    draw(mat, a.p3, Color::red());
  }
}

}

#endif //OCCUPANCY_GRID_DRAW_H
