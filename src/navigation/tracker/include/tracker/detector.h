#ifndef TRACKER_APRILTAGS_H
#define TRACKER_APRILTAGS_H

#include <sys/types.h>
#include <stdint.h>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <opencv2/opencv.hpp>

#include <tracker/camera/camera_info.h>
#include <tracker/tag.h>

typedef struct apriltag_family apriltag_family_t;
typedef struct apriltag_detector apriltag_detector_t;
typedef struct image_u8 image_u8_t;
typedef struct apriltag_detection apriltag_detection_t;

typedef tf2::Stamped<tf2::Transform> StampedTransform;

namespace tracker
{
  class Detector
  {
    public:
      Detector(CameraInfo camera_info, uint8_t *buffer, TagsVector *tags);
      ~Detector();

      StampedTransform getRelativeTransform(double tag_size, apriltag_detection_t *detection, ros::Time stamp);
      void detect(ros::Time stamp);
      //tf2::Transform getPose(apriltag_detection_t *detection);
      uchar* getBuffer();
      void drawDetection(apriltag_detection_t *detection);

    private:
      apriltag_family_t *family;
      apriltag_detector_t *detector;
      image_u8_t *at_image;
      CameraInfo camera_info;
      cv::Mat cv_image, map1, map2;
      TagsVector *tags;
  };
}

#endif //TRACKER_APRILTAGS_H
