#ifndef TRACKER_APRILTAGS_H
#define TRACKER_APRILTAGS_H

#include <sys/types.h>
#include <stdint.h>
#include <vector>
#include <tracker/camera/camera_info.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/opencv.hpp>

typedef struct apriltag_family apriltag_family_t;
typedef struct apriltag_detector apriltag_detector_t;
typedef struct image_u8 image_u8_t;
typedef struct apriltag_detection apriltag_detection_t;

typedef tf2::Stamped<tf2::Transform> StampedTransform;

namespace tracker
{

  class AprilTag
  {
  public:
    AprilTag(int family, int id, int size, tf2::Transform placement);

    const int family;
    const int id;
    const int size;
    const tf2::Transform placement;

  private:
    int readings_start, readings_end;
    std::vector<StampedTransform> readings;
  };

    class AprilTagDetector
    {
      public:
        AprilTagDetector(CameraInfo camera_info, uint8_t *buffer);
        ~AprilTagDetector();

        void addTag(int family, int id, int size, tf2::Transform placement);
        StampedTransform getRelativeTransform(apriltag_detection_t detection);
        void detect();
        tf2::Transform getPose(apriltag_detection_t detection);
        uchar* getBuffer();
        void drawDetection(apriltag_detection_t *detection);

      private:
        apriltag_family_t *family;
        apriltag_detector_t *detector;
        image_u8_t *at_image;
        cv::Mat cv_image;
        std::vector<AprilTag> tags;
        CameraInfo camera_info;
    };
}

#endif //TRACKER_APRILTAGS_H
