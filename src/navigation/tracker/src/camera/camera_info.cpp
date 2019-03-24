#include <tracker/camera/camera_info.h>

using namespace tracker;

CameraInfo::CameraInfo(std::string path) :
    name("default_camera"),
    path(path),
    width(1280),
    height(720),
    camera_matrix(3,3),
    distortion_matrix(5,1)
{}

CameraInfo::CameraInfo(std::string name, std::string path) :
  name(name),
  path(path),
  width(1280),
  height(720),
  camera_matrix(3,3),
  distortion_matrix(5,1)
{}

CameraInfo::CameraInfo(std::string name, std::string path, uint width, uint height,
                       cv::Mat_<double> camera_matrix, cv::Mat_<double> distortion_matrix) :
  name(name),
  path(path),
  width(width),
  height(height),
  camera_matrix(camera_matrix),
  distortion_matrix(distortion_matrix)
{}

double CameraInfo::getFx()
{
  return camera_matrix.at<double>(0,0);
}

double CameraInfo::getFy()
{
  return camera_matrix.at<double>(1,1);
}

double CameraInfo::getCx()
{
  return camera_matrix.at<double>(0,2);
}

double CameraInfo::getCy()
{
  return camera_matrix.at<double>(1,2);
}

