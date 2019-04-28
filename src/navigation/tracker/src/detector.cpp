#include <tracker/detector.h>

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
Detector::Detector(CameraInfo camera_info, uint8_t *buffer, TagsVector *tags):
    camera_info(camera_info), tags(tags),
    cv_image(camera_info.height, camera_info.width, CV_8UC1, buffer),
    map1(camera_info.height, camera_info.width, CV_16SC2),
    map2(camera_info.height, camera_info.width, CV_16UC1)
{
  //family = tag25h10_create();
  family = tag36h11_create();
  detector = apriltag_detector_create();
  detector->quad_decimate = 6.0;      // Default = 2.0, most of semester was 3.0
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

void Detector::detect(ros::Time stamp)
{
  cv::rotate(cv_image, cv_image, cv::ROTATE_180);
  cv::Mat tmp = cv_image.clone();
  cv::remap(tmp, cv_image, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  zarray_t *detections = apriltag_detector_detect(detector, at_image);

  // TODO get transforms
  for (int i = 0; i < zarray_size(detections); i++)
  {
    apriltag_detection_t *detection;
    zarray_get(detections, i, &detection);
    for (int j = 0; j < tags->size(); j++)
    {
      if (detection->id == (*tags)[j].getID())
      {
        (*tags)[j].addRelativeTransform(getRelativeTransform((*tags)[j].getSize(), detection, stamp));
      }
    }
  }

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

StampedTransform Detector::getRelativeTransform(double tag_size, apriltag_detection_t *detection, ros::Time stamp)
{

  double s = tag_size / 2.0;
  std::vector<cv::Point3d> tag_points;
  tag_points.emplace_back(-s, -s, 0);
  tag_points.emplace_back( s, -s, 0);
  tag_points.emplace_back( s,  s, 0);
  tag_points.emplace_back(-s,  s, 0);

  std::vector<cv::Point2d> image_points;
  for (int i = 0; i < 4; i++)
  {
    image_points.emplace_back(detection->p[i][0], detection->p[i][1]);
  }

  cv::Mat rvec, tvec;
  // TODO make sure the properties are accurate
  // already undistorted the image, so distortion parameter is an empty matrix
  cv::Mat empty;
  cv::solvePnP(tag_points, image_points, camera_info.camera_matrix, empty, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d R;
  R << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2);

  Eigen::Matrix4d T;
  T.topLeftCorner(3, 3) = R;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;

  // Build transform
  StampedTransform tag_transform;
  tag_transform.stamp_ = stamp;
  tag_transform.setOrigin(tf2::Vector3(T(0, 3), T(1, 3), T(2, 3)));    // TODO double check this
  Eigen::Quaternion<double> rotation_q = Eigen::Quaternion<double>(R); // TODO fix duplication
  tf2::Quaternion q;
  q.setX(rotation_q.x());
  q.setY(rotation_q.y());
  q.setZ(rotation_q.z());
  q.setW(rotation_q.w());
  tag_transform.setRotation(q);

  //printf("%i, %f, %f, %f, %f\n", detection->id, T(0, 3), T(1, 3), T(2, 3), std::sqrt(T(0, 3)*T(0, 3) + T(2, 3)*T(2, 3)));

  return tag_transform;
}
