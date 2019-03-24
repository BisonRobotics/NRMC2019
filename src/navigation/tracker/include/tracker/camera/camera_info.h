#ifndef TRACKER_CAMERA_PARAMETERS_H
#define TRACKER_CAMERA_PARAMETERS_H

#include <stdint.h>
#include <sys/types.h>
#include <string>
#include <opencv2/core.hpp>

namespace tracker
{
  class CameraInfo
  {
  public:
    CameraInfo(const CameraInfo &info) = default;
    explicit CameraInfo(std::string path);
    CameraInfo(std::string name, std::string path);
    CameraInfo(std::string name, std::string path, uint width, uint height,
               cv::Mat_<double> camera_matrix, cv::Mat_<double> distortion_matrix);
    const std::string name, path;
    const uint width, height;
    const cv::Mat_<double> camera_matrix, distortion_matrix;

    double getFx();
    double getFy();
    double getCx();
    double getCy();
  };
}

#endif //TRACKER_CAMERA_PARAMETERS_H
