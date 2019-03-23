#ifndef TRACKER_TRANSFORMS_CACHE_H
#define TRACKER_TRANSFORMS_CACHE_H

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_tracker/tag.h>

namespace tracker
{

  class TransformsCache
  {
  public:
    TransformsCache() {};
    TransformsCache(ros::NodeHandle nh); // TODO test with ROS tests
    static void initializeTagVector(std::vector<Tag> *tag_info);

    tf2::Transform camera_optical_to_camera_mount;
    tf2::Transform dynamixel_to_base_link;
  };

}

#endif //TRACKER_TRANSFORMS_CACHE_H
