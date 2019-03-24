#ifndef TRACKER_TAG_H
#define TRACKER_TAG_H

#include <list>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>


namespace tracker
{
  class Tag;

  bool operator == (const tracker::Tag &lhs, const tracker::Tag &rhs);
  bool operator != (const tracker::Tag &lhs, const tracker::Tag &rhs);

  typedef tf2::Stamped<tf2::Transform> StampedTransform;
  typedef std::vector<Tag> TagsVector;

  class Tag
  {
  public:
    static void init(uint list_size, ros::Duration max_dt, bool find_tfs);
    static std::vector<Tag> getTags();
    static uint getListSize();
    static ros::Duration getMaxDt();
    static bool isInitialized();
    static tf2::Transform findMapToTag(int id);
    static void updateMapToTagTfs();

    Tag(Tag *tag);
    Tag(const Tag &tag);
    Tag(int id, double tag_size, tf2::Transform map_to_tag = tf2::Transform());

    void updateRelativeTransform(StampedTransform relative_transform);
    void processDetection(tf2::Transform servo, ros::Time stamp);

    // Accessors
    int getID() const;
    tf2::Transform getMapToTag() const;
    unsigned int getSeq() const;
    double getSize() const;
    long getTransformsSize() const;
    StampedTransform getRelativeTransform() const;
    std::vector<StampedTransform> getTransforms() const;
    StampedTransform getMostRecentTransform() const;

  private:
    static bool initialized;
    static uint list_size;
    static ros::Duration max_dt;
    static std::vector<Tag> tags;

    int id;
    unsigned int seq;
    double tag_size;
    tf2::Transform map_to_tag;
    StampedTransform relative_transform;
    std::list<tf2::Stamped<tf2::Transform>> transforms;
  };
}

#endif //TRACKER_TAG_H
