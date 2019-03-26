#ifndef TRACKER_TAG_H
#define TRACKER_TAG_H

#include <list>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>


namespace tracker
{
  class Tag;

  // Comparison operators for unit tests
  bool operator == (const tracker::Tag &lhs, const tracker::Tag &rhs);
  bool operator != (const tracker::Tag &lhs, const tracker::Tag &rhs);

  // Typedefs
  typedef tf2::Stamped<tf2::Transform> StampedTransform;
  typedef std::list<StampedTransform> StampedTransformList;
  typedef std::vector<Tag> TagsVector;

  class Tag
  {
  public:
    // Constructors
    Tag(Tag *tag);
    Tag(const Tag &tag);
    Tag(int id, double tag_size);

    // Methods
    void addRelativeTransform(StampedTransform relative_transform);
    void addStepperTransform(StampedTransform stepper_transform);
    geometry_msgs::PoseStamped estimatePose();

    // Accessors
    int getID() const;
    unsigned int getSeq() const;
    double getSize() const;
    long getTransformsSize() const;
    tf2::Transform getMapToTag() const;
    StampedTransform getMostRecentRelativeTransform() const;
    std::vector<StampedTransform> getRelativeTransforms() const;
    uint getRelativeTransformsSize() const;
    bool relativeTransformUpdated() const;
    bool stepperTransformUpdated() const;

  private:
    // Data
    int id;
    unsigned int seq;
    double tag_size;
    bool relative_transform_updated, stepper_transform_updated;
    tf2::Transform tag_to_map, base_link_to_camera_base;
    std::list<StampedTransform> relative_transforms, stepper_transforms;

  public:
    // Static methods
    static void init(uint list_size, ros::Duration max_dt, bool find_tfs);
    static std::vector<Tag> getTags();
    static uint getListSize();
    static ros::Duration getMaxDt();
    static bool isInitialized();
    static tf2::Transform findMapToTag(int id);
    static void updateMapToTagTfs();
    static void setTransformCaches(TagsVector *tags, std::string prefix); // Should be left or right
    static void clearFlags(TagsVector *tags);
    static void addTransformToList(StampedTransformList *list, const StampedTransform &transform);

  private:
    // Static data
    static bool initialized;
    static uint list_size;
    static ros::Duration max_dt;
    static std::vector<Tag> tags;
  };
}

#endif //TRACKER_TAG_H
