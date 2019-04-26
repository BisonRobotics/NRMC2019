#include <tracker/camera/ocam_camera.h>
#include <fstream>

tracker::OCamCamera::OCamCamera(CameraInfo info, uint fps, uint brightness, uint exposure):
                                info(info), fps(fps), brightness(brightness), exposure(exposure)
{
  // Make sure camera is exists
  std::ifstream f(info.path);
  if (!f.good())
  {
    throw std::runtime_error("Unable to find camera");
  }

  // Set camera parameters
  camera = new ocam::Camera(info.path.c_str());
  camera->set_format(info.width, info.height, ocam::fourcc_to_pixformat('G','R','E','Y'), 1, fps);
  setBrightness(brightness);
  setExposure(exposure);
  sequence = 0;

  // Make sure all parameters were set properly
  camera->get_current_format(cam_format);
  if (cam_format.height != info.height ||
      cam_format.width != info.width ||
      cam_format.rate_denominator != fps ||
      getBrightness() != brightness ||
      getExposure() != exposure)
  {
    throw std::runtime_error("Device parameters don't match those requested");
  }
}

tracker::OCamCamera::~OCamCamera()
{
  delete(camera);
}

void tracker::OCamCamera::start()
{
  if (!camera->start())
  {
    throw std::runtime_error("Unable to start camera");
  }
}

void tracker::OCamCamera::stop()
{
  if (!camera->stop())
  {
    throw std::runtime_error("Unable to stop camera");
  }
}

// Blocking function
void tracker::OCamCamera::getFrame(unsigned char *image_buffer)
{
  int size = camera->get_frame(image_buffer, cam_format.image_size, 1);
  if (size == -1)
  {
    throw std::runtime_error("Unable to retrieve frame");
  }
  sequence++;
}

uint tracker::OCamCamera::getBrightness()
{
  int brightness = camera->get_control("Brightness");
  if (brightness == -1)
  {
    throw std::runtime_error("Unable to retrieve brightness from device");
  }
  return (uint)brightness;
}

void tracker::OCamCamera::setBrightness(uint brightness)
{
  if (!camera->set_control("Brightness", brightness))
  {
    throw std::runtime_error("Unable to set brightness for device");
  }
}

uint tracker::OCamCamera::getExposure()
{
  int exposure = camera->get_control("Exposure (Absolute)");
  if (exposure == -1)
  {
    throw std::runtime_error("Unable to retrieve exposure from device");
  }
  return (uint) exposure;
}

void tracker::OCamCamera::setExposure(uint exposure)
{
  if (!camera->set_control("Exposure (Absolute)", exposure))
  {
    throw std::runtime_error("Unable to set brightness for device");
  }
}

uint tracker::OCamCamera::getSequence()
{
  return sequence;
}

uint tracker::OCamCamera::getWidth()
{
  return info.width;
}

uint tracker::OCamCamera::getHeight()
{
  return info.height;
}

tracker::CameraInfo tracker::OCamCamera::getInfo()
{
  return info;
}

std::string tracker::OCamCamera::getName()
{
  return info.name;
}

void tracker::OCamCamera::reboot()
{
  delete(camera);

  // Make sure camera is exists
  std::ifstream f(info.path);
  if (!f.good())
  {
    throw std::runtime_error("Unable to find camera");
  }

  // Set camera parameters
  camera = new ocam::Camera(info.path.c_str());
  camera->set_format(info.width, info.height, ocam::fourcc_to_pixformat('G','R','E','Y'), 1, fps);
  setBrightness(brightness);
  setExposure(exposure);
  sequence = 0;

  // Make sure all parameters were set properly
  camera->get_current_format(cam_format);
  if (cam_format.height != info.height ||
      cam_format.width != info.width ||
      cam_format.rate_denominator != fps ||
      getBrightness() != brightness ||
      getExposure() != exposure)
  {
    throw std::runtime_error("Device parameters don't match those requested");
  }
}
