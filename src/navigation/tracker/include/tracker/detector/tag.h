#ifndef TRACKER_TAG_H
#define TRACKER_TAG_H

#include <list>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>

namespace tracker
{
  typedef tf2::Stamped<tf2::Transform> StampedTransform;

  class Tag
  {
  public:
    Tag(int id, int list_size, double tag_size, ros::Duration max_dt);

    void addDetection(cv::Point2d center, tf2::Transform camera_to_tag,
                      tf2::Transform servo, ros::Time stamp);

    int getID();
    static tf2::Transform findMapToTag(int id);
    tf2::Transform getMapToTag();
    unsigned int getSeq();
    double getSize();
    long getTransformsSize();
    std::vector<StampedTransform> getTransforms();
    StampedTransform getMostRecentTransform();

  private:
    int id;
    unsigned int seq;
    double tag_size;
    int list_size;
    ros::Duration max_dt;
    tf2::Transform map_to_tag;
    std::list<tf2::Stamped<cv::Point2d>> centers;
    std::list<tf2::Stamped<tf2::Transform>> transforms;
  };

}

#endif //TRACKER_TAG_H
