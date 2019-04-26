#include <tracker/camera/image_camera.h>
#include <fstream>
#include <opencv2/opencv.hpp>

tracker::ImageCamera::ImageCamera(const std::string &image_path, uint width, uint height,
        uint fps, uint brightness, uint exposure)
{
  this->seq = 0;
  this->width = width;
  this->height = height;
  this->fps = fps;
  this->brightness = brightness;
  this->exposure = exposure;
  this->image = imread(image_path, cv::IMREAD_UNCHANGED);
  if (!image.data)
  {
    throw std::runtime_error("Unable to load image");
  }
}

tracker::ImageCamera::~ImageCamera()
{
}

void tracker::ImageCamera::start()
{

}

void tracker::ImageCamera::stop()
{

}

// Blocking function
void tracker::ImageCamera::getFrame(unsigned char *image_buffer)
{
  memcpy(image.data, image_buffer, image.total()*image.elemSize());
}

uint tracker::ImageCamera::getBrightness()
{
  return brightness;
}

void tracker::ImageCamera::setBrightness(uint brightness)
{
  this->brightness = brightness;
}

uint tracker::ImageCamera::getExposure()
{
  return exposure;
}

void tracker::ImageCamera::setExposure(uint exposure)
{
  this->exposure = exposure;
}

uint tracker::ImageCamera::getSequence()
{
 return seq;
}

void tracker::ImageCamera::reboot()
{

}

uint tracker::ImageCamera::getWidth()
{
  return 0;
}

uint tracker::ImageCamera::getHeight()
{
  return 0;
}
