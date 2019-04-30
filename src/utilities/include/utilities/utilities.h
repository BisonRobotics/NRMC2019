#ifndef UTILITIES_UTILITIES_H
#define UTILITIES_UTILITIES_H

#include <geometry_msgs/Pose2D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace utilities
{
  geometry_msgs::Pose2D toPose2D(const geometry_msgs::TransformStamped &transform);
  geometry_msgs::Pose2D toPose2D(const geometry_msgs::Transform &transform);
  geometry_msgs::Pose2D toPose2D(const tf2::Transform &transform);
}

#endif //UTILITIES_UTILITIES_H
