#include <tracker/detector/detector.h>

extern "C"
{
#include "apriltags/apriltag.h"
#include "apriltags/tags/tag16h6.h"
#include "apriltags/tags/tag25h10.h"
#include "apriltags/tags/tag36h11.h"
#include "apriltags/common/getopt.h"
#include "apriltags/apriltag_pose.h"
}


using namespace tracker;

// TODO retry tag16h6 with a check for tag size
// TODO generate tag36hX with a Hamming distance X
//  greater than 11 (less tags, better accuracy)


// Note: As long as the tag is detected at the quad_decimate level, if refine_edges is enabled, it will
// go through and refine the edges on the full resolution image.
Detector::Detector(CameraInfo camera_info, uint8_t *buffer):
    camera_info(camera_info),
    cv_image(camera_info.height, camera_info.width, CV_8UC1, buffer),
    map1(camera_info.height, camera_info.width, CV_16SC2),
    map2(camera_info.height, camera_info.width, CV_16UC1)
{
  //family = tag25h10_create();
  family = tag36h11_create();
  detector = apriltag_detector_create();
  detector->quad_decimate = 4.0;      // Default = 2.0
  detector->quad_sigma = 0.0;         // Default = 0.0
  detector->refine_edges = 1;         // Default = 1
  detector->decode_sharpening = 0.25; // Default = 0.25
  detector->nthreads = 4;
  detector->debug = 0;
  apriltag_detector_add_family(detector, family);
  at_image = new image_u8
  {
    (int32_t)camera_info.width,
    (int32_t)camera_info.height,
    (int32_t)camera_info.width,
    buffer
  };

  cv::initUndistortRectifyMap(
      camera_info.camera_matrix,
      camera_info.distortion_matrix,
      cv::Mat_<double>::eye(3,3),
      camera_info.camera_matrix,
      cv::Size(camera_info.width, camera_info.height),
      map1.type(), map1, map2);
}

uchar* Detector::getBuffer()
{
  return cv_image.data;
}

void Detector::drawDetection(apriltag_detection_t *detection)
{
  line(cv_image, cv::Point((int)detection->p[0][0], (int)detection->p[0][1]),
                 cv::Point((int)detection->p[1][0], (int)detection->p[1][1]),
                 cv::Scalar(0xff), 2);
  line(cv_image, cv::Point((int)detection->p[0][0], (int)detection->p[0][1]),
                 cv::Point((int)detection->p[3][0], (int)detection->p[3][1]),
                 cv::Scalar(0xff), 2);
  line(cv_image, cv::Point((int)detection->p[1][0], (int)detection->p[1][1]),
                 cv::Point((int)detection->p[2][0], (int)detection->p[2][1]),
                 cv::Scalar(0xff), 2);
  line(cv_image, cv::Point((int)detection->p[2][0], (int)detection->p[2][1]),
                 cv::Point((int)detection->p[3][0], (int)detection->p[3][1]),
                 cv::Scalar(0xff), 2);
  cv::String text = std::to_string(detection->id);
  const int font_face = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  const double font_scale = 1.0;
  int baseline;
  cv::Size text_size = cv::getTextSize(text, font_face, font_scale, 2, &baseline);
  putText(cv_image, text,
          cv::Point((int)detection->c[0]-text_size.width/2,
                    (int)detection->c[1]+text_size.height/2),
          font_face, font_scale, cv::Scalar(0x80), 2);
}

void Detector::detect()
{
  cv::rotate(cv_image, cv_image, cv::ROTATE_180);
  cv::Mat tmp = cv_image.clone();
  cv::remap(tmp, cv_image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  zarray_t *detections = apriltag_detector_detect(detector, at_image);

  // TODO get transforms

  // Draw detection outlines
  for (int i = 0; i < zarray_size(detections); i++)
  {
    apriltag_detection_t *detection;
    zarray_get(detections, i, &detection);
    drawDetection(detection);
  }
  apriltag_detections_destroy(detections);
}

Detector::~Detector()
{
  tag25h10_destroy(family);
  //tag36h11_destroy(tf);
  apriltag_detector_destroy(detector);
  delete(at_image);
}

tf2::Stamped<tf2::Transform> Detector::getRelativeTransform(apriltag_detection_t detection)
{
  /*at_info.fx = ;
  at_info.cx = ;
  at_info.fy = ;
  at_info.cy = ;*/

  apriltag_detection_info_t info;
  info.tagsize = 0.2;
  info.fx = camera_info.getFx();
  info.fy = camera_info.getFy();
  info.cx = camera_info.getCx();
  info.cy = camera_info.getCy();
  info.det = &detection;

  apriltag_pose_t pose;
  pose.t = matd_create(4, 4);
  pose.R = matd_create(4, 4);

  estimate_tag_pose(&info, &pose);


  return tf2::Stamped<tf2::Transform>();
}
