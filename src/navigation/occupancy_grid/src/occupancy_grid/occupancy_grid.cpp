#include <occupancy_grid/occupancy_grid.h>
#include <arena.h>

using namespace occupancy_grid;

void OccupancyGrid::show()
{
  namedWindow( "OGrid Display", cv::WINDOW_NORMAL );
  imshow( "OGrid Display", *this);
  cv::waitKey(0);
}

void OccupancyGrid::write()
{
  Mat_<double> image;
  this->copyTo(image);
  image = 255.0 - 255.0 * image;
  imwrite("occupancy_grid.png", image);
}

void OccupancyGrid::max(OccupancyGrid &a, OccupancyGrid &b, OccupancyGrid &out)
{
  for (int i = 0; i < a.rows; i++)
  {
    for (int j = 0; j < a.cols; j++)
    {
      out(i,j) = std::max(a(i,j), b(i,j));
    }
  }
}

void OccupancyGrid::draw(Point &a)
{
  const cv::Scalar_<double> fill = cv::Scalar_<double>(1.0);
  cv::drawMarker((*this), a.imgTf(), fill, cv::MARKER_CROSS);
}

void OccupancyGrid::draw(Line &a)
{
  const cv::Scalar_<double> fill = cv::Scalar_<double>(1.0);
  cv::line((*this), a.p0.imgTf(), a.p1.imgTf(), fill);
}

void OccupancyGrid::draw(Circle &a)
{
  const cv::Scalar_<double> fill = cv::Scalar_<double>(1.0);
  cv::circle((*this), a.imgTfP(), a.imgTfR(), fill, -1);
}

void OccupancyGrid::inflate(OccupancyGrid &in, OccupancyGrid &out, double cutoff)
{
  cv::blur(in, out, cv::Size(100, 100));
  for (int i = 0; i < out.rows; i++)
  {
    for (int j = 0; j < out.cols; j++)
    {
      if (out(i,j) > cutoff)
      {
        out(i,j) = 1.0;
      }
      else
      {
        out(i,j) = out(i,j) / cutoff;
      }
    }
  }
}

