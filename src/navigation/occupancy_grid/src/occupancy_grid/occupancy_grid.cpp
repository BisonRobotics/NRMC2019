#include <occupancy_grid/occupancy_grid.h>
#include <occupancy_grid/arena_dimensions.h>

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

void OccupancyGrid::max(OccupancyGrid const &a, OccupancyGrid const &b, OccupancyGrid *out)
{
  for (int i = 0; i < a.rows; i++)
  {
    for (int j = 0; j < a.cols; j++)
    {
      (*out)(i,j) = std::max(a(i,j), b(i,j));
    }
  }
}

void OccupancyGrid::draw(Point const &a)
{
  const cv::Scalar_<double> fill = cv::Scalar_<double>(1.0);
  cv::drawMarker((*this), a.imgTf(), fill, cv::MARKER_CROSS);
}


void OccupancyGrid::draw(Line const &a, int thickness=1)
{
  const cv::Scalar_<double> fill = cv::Scalar_<double>(1.0);
  std::cout << "P1:[" << a.p0.imgTf().x << " " << a.p0.imgTf().y << "] ";
  std::cout << "P2:[" << a.p1.imgTf().x << " " << a.p1.imgTf().y << "]" << std::endl;
  cv::line((*this), a.p0.imgTf(), a.p1.imgTf(), fill, thickness);
}

void OccupancyGrid::draw(Circle const &a)
{
  const cv::Scalar_<double> fill = cv::Scalar_<double>(1.0);
  cv::circle((*this), a.imgTfP(), a.imgTfR(), fill, -1);
}

void OccupancyGrid::inflate(OccupancyGrid const &in, OccupancyGrid *out,
                            double cutoff, int kernel_size, int passes)
{
  // Averaging filter
  cv::blur(in, (*out), cv::Size(kernel_size, kernel_size));
  for (int i = 0; i < passes; i++)
  {
    cv::blur((*out), (*out), cv::Size(kernel_size, kernel_size));
  }

  // Normalize and threshold
  double min, max;
  cv::minMaxLoc((*out), &min, &max);
  for (int i = 0; i < out->rows; i++)
  {
    for (int j = 0; j < out->cols; j++)
    {
      (*out)(i,j) = (*out)(i,j) / max;
      if ((*out)(i,j) > cutoff)
      {
        (*out)(i,j) = 1.0;
      }
      else
      {
        (*out)(i,j) = (*out)(i,j) / cutoff;
      }
    }
  }
}

