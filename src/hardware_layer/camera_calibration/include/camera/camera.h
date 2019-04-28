#ifndef TRACKER_CAMERA_H
#define TRACKER_CAMERA_H

#include <camera/camera_info.h>

namespace tracker
{
  class Camera
  {
    public:
      virtual void start() = 0;
      virtual void stop() = 0;
      virtual void reboot() = 0;
      virtual void getFrame(unsigned char* image_buffer) = 0;
      virtual void setBrightness(uint brightness) = 0;
      virtual uint getBrightness() = 0;
      virtual void setExposure(uint exposure) = 0;
      virtual uint getExposure() = 0;
      virtual uint getSequence() = 0;
      virtual uint getWidth() = 0;
      virtual uint getHeight() = 0;
      virtual CameraInfo getInfo() = 0;
      virtual std::string getName() = 0;
  };
}

#endif //TRACKER_CAMERA_H
