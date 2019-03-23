#ifndef TRACKER_TAG_H
#define TRACKER_TAG_H

#include <list>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>

#include <tracker/detector/transform.h>

namespace tracker
{

  class Tag
  {
  public:
    // TODO some of this should be made private
    Tag(int id, int priority, double size);

    // Thread safe
    void addTransform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf,
                      tf2::Stamped<tf2::Transform> servo_tf, unsigned int seq);
    void setMapToTagTf(tf2::Transform tf);
    int getID();
    //std::string getFrameID(); Not thread safe
    cv::Point getDetectionCenter();
    unsigned int getSeq();
    double getGoodness();
    double getPriority();
    double getSize();
    long getTransformsListSize();

    // Helper methods for calculating the moving average transform
    // These were copied from pose_estimate filter, which is a sin, but we're crunched for time
    static void flushOldTransforms(std::list<apriltag_tracker::Transform> *transforms, ros::Time current_time,
                                   ros::Duration max_dt);
    static void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw);
    static double getTheta(tf2::Quaternion orientation);
    static tf2::Quaternion getAverageOrientation(std::list<apriltag_tracker::Transform> &transforms, int filter_size);
    static tf2::Vector3 getAveragePosition(std::list<apriltag_tracker::Transform> &transforms, int filter_size);

    // Transform getters
    std::vector<Transform> getTransforms();
    Transform getMovingAverageTransform();
    Transform getMovingAverageTransform(int number_of_transforms);
    Transform getMovingAverageTransform(int number_of_transforms, double max_dt);
    Transform getMedianMovingAverageTransform();
    Transform getMedianFilteredTransform();
    Transform getMostRecentTransform();
    tf2::Transform getMapToTagTf();
    double getAngleFromCenter(int number_of_transforms);
    double getAngleFromCenter();

  private:
    boost::mutex *mutex;
    int id;
    CompareType compare_mode;
    unsigned int seq;
    double goodness;
    double priority;
    double size;
    //std::string frame_id;

    int list_size;
    tf2::Transform map_to_tag_tf;
    std::list<apriltag_tracker::Transform> transforms;
  };

}

#endif //TRACKER_TAG_H
