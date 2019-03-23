#include <tracker/detector/transform.h>

using namespace tracker;

Transform::Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf,
                     tf2::Transform map_to_tag_tf, CompareType* compare_mode)
    : Transform::Transform(detection, tag_tf, servo_tf, map_to_tag_tf)
{
  this->compare_mode = compare_mode;
}

Transform::Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf,
                     tf2::Transform map_to_tag_tf)
{
  this->tag_tf = tag_tf;
  this->servo_tf = servo_tf;
  this->detection = detection;
  this->map_to_tag_tf = map_to_tag_tf;
  this->compare_mode = new CompareType;
  *(this->compare_mode) = CompareType::invalid;

  double tmp1, tmp2;
  getRPY(tag_tf.getRotation(), tmp1, this->tag_theta, tmp2);
}

void Transform::getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 matrix;
  matrix.setRotation(q);
  matrix.getRPY(roll, pitch, yaw);
}

tf2::Stamped<tf2::Transform> Transform::getTagTf()
{
  return this->tag_tf;
}

tf2::Stamped<tf2::Transform> Transform::getServoTf()
{
  return this->servo_tf;
}

tf2::Transform Transform::getMapToTagTf()
{
  return this->map_to_tag_tf;
}

double Transform::getTagTheta()
{
  return this->tag_theta;
}

bool Transform::operator<(Transform tag_tf)
{
  if (*compare_mode == CompareType::distance)
  {
    tf2::Vector3 this_origin = this->tag_tf.getOrigin();
    tf2::Vector3 that_origin = tag_tf.getTagTf().getOrigin();
    double this_tag = this_origin.getX() * this_origin.getX() + this_origin.getZ() * this_origin.getZ();
    double that_tag = that_origin.getX() * that_origin.getX() + that_origin.getZ() * that_origin.getZ();
    return this_tag < that_tag;
  }
  else if (*compare_mode == CompareType::theta)
  {
    return this->getTagTheta() < tag_tf.getTagTheta();
  }
  else
  {
    throw std::runtime_error("Compare mode is in an unknown state");
  }
}

cv::Point Transform::getDetectionCenter()
{
  return detection.cxy;
}

TagDetection Transform::getDetection()
{
  return detection;
}