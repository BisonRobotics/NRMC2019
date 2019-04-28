#ifndef TRACKER_OCAM_CAMERA_H
#define TRACKER_OCAM_CAMERA_H

#include <string>
#include <camera/camera.h>
#include <camera/ocam/camera.hpp>

namespace tracker
{
    class OCamCamera : public Camera
    {
      public:
        OCamCamera(CameraInfo info, uint fps, uint brightness, uint exposure);
        ~OCamCamera();

        void start() override;
        void stop() override;
        void reboot() override;
        void getFrame(unsigned char* image_buffer) override;
        uint getBrightness() override;
        void setBrightness(uint brightness) override;
        uint getExposure() override;
        void setExposure(uint exposure) override;
        uint getSequence() override;
        uint getWidth() override;
        uint getHeight() override;
        CameraInfo getInfo() override;
        std::string getName() override;


    private:
        ocam::Camera *camera;
        ocam::camera_format cam_format;
        uint sequence;
        CameraInfo info;
        uint brightness, exposure, fps;
    };
}

#endif //TRACKER_OCAM_CAMERA_H
