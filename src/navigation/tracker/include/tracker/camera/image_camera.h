#ifndef TRACKER_IMAGE_CAMERA_H
#define TRACKER_IMAGE_CAMERA_H

#include <string>
#include <tracker/camera/camera.h>
#include <opencv2/opencv.hpp>


namespace tracker
{
    class ImageCamera : public Camera
    {
      public:
        ImageCamera(const std::string &image_path, uint width, uint height,
                uint fps, uint brightness, uint exposure);
        ~ImageCamera();

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

      private:
        uint seq, width, height, fps, brightness, exposure;
        cv::Mat image;
    };
}

#endif //TRACKER_IMAGE_CAMERA_H
