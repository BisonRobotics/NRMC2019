#ifndef TRACKER_TRANSFORM_H
#define TRACKER_TRANSFORM_H

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>
//#include <TagDetection.h>


namespace tracker
{

  enum class CompareType
  {
    theta,
    distance,
    invalid
  };

  class Transform
  {
  public:
    /*Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf,
              tf2::Transform map_to_tag_tf, CompareType* compare_mode);
    Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf,
              tf2::Transform map_to_tag_tf);*/

    tf2::Stamped<tf2::Transform> getTagTf();
    tf2::Stamped<tf2::Transform> getServoTf();
    tf2::Transform getMapToTagTf();
    double getTagTheta();
    cv::Point getDetectionCenter();
    TagDetection getDetection();
    bool operator<(Transform tag_tf);

    static void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw);

  private:
    tf2::Stamped<tf2::Transform> tag_tf;
    tf2::Stamped<tf2::Transform> servo_tf;
    tf2::Transform map_to_tag_tf;
    TagDetection detection;
    double tag_theta;
    CompareType* compare_mode;
  };

}

#endif //TRACKER_TRANSFORM_H
