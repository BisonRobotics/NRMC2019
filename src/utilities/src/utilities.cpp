#include <utilities/utilities.h>

using namespace utilities;

geometry_msgs::Pose2D utilities::toPose2D(const geometry_msgs::TransformStamped &transform_msg)
{
  tf2::Transform transform;
  tf2::fromMsg(transform_msg.transform, transform);
  return toPose2D(transform);}

geometry_msgs::Pose2D utilities::toPose2D(const geometry_msgs::Transform &transform_msg)
{
  tf2::Transform transform;
  tf2::fromMsg(transform_msg, transform);
  return toPose2D(transform);
}

geometry_msgs::Pose2D utilities::toPose2D(const tf2::Transform &transform)
{
  // Make sure that the robot rotation is valid
  auto Q1 = transform.getRotation();
  double Q1m = Q1.x()*Q1.x() + Q1.y()*Q1.y() + Q1.z()*Q1.z() + Q1.w()*Q1.w();
  if (std::abs(Q1m - 1.0) > 1.0e-6)
  {
    throw std::runtime_error("[utilities::toPose2D]: Robot orientation isn't normalized");
  }

  tf2::Matrix3x3 R(transform.getRotation());
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);

  geometry_msgs::Pose2D pose;
  pose.x = transform.getOrigin().x();
  pose.y = transform.getOrigin().y();
  pose.theta = yaw;
  return pose;
}
