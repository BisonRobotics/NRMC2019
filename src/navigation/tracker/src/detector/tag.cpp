#include <tracker/detector/tag.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace tracker
{
bool operator == (const Tag &lhs, const Tag &rhs)
{
  return lhs.getID()         == rhs.getID()         &&
         lhs.getSeq()        == rhs.getSeq()        &&
         lhs.getSize()       == rhs.getSize()       &&
         lhs.getMapToTag()   == rhs.getMapToTag()   &&
         lhs.getTransforms() == rhs.getTransforms();
}

bool operator != (const Tag &lhs, const Tag &rhs)
{
  return !(lhs.getID()         == rhs.getID()         &&
           lhs.getSeq()        == rhs.getSeq()        &&
           lhs.getSize()       == rhs.getSize()       &&
           lhs.getMapToTag()   == rhs.getMapToTag()   &&
           lhs.getTransforms() == rhs.getTransforms());
}
}

using namespace tracker;
using std::vector;


// Initialize static members
std::vector<Tag> Tag::tags;
bool Tag::initialized = false;
uint Tag::list_size = 0;
ros::Duration Tag::max_dt;


void Tag::init(uint list_size, ros::Duration max_dt, bool find_tfs)
{
  Tag::list_size = list_size;
  Tag::max_dt = max_dt;
  if (find_tfs)
  {
    updateMapToTagTfs();
  }
  else
  {
    ROS_WARN("Map to tag tfs might be uninitialized");
  }
  Tag::initialized = true;
}

Tag::Tag(const Tag &tag) :
    id(tag.id), seq(tag.seq), tag_size(tag.tag_size),
    map_to_tag(tag.map_to_tag)
{

}

Tag::Tag(Tag *tag) :
    id(tag->id), seq(tag->seq), tag_size(tag->tag_size),
    map_to_tag(tag->map_to_tag)
{

}

Tag::Tag(int id, double tag_size, tf2::Transform map_to_tag) :
    id(id), tag_size(tag_size), map_to_tag(map_to_tag), seq(0)
{
  // Add tag to global vector if it doesn't already exist
  bool tag_exists = false;
  for (int i = 0; i < tags.size(); i++)
  {
    if (tags[i].getID() == id)
    {
      tag_exists = true;
      break;
    }
  }
  if (!tag_exists)
  {
    tags.emplace_back(this);
  }
}

tf2::Transform Tag::findMapToTag(int id)
{
  std::string tag_name = "tag" + std::to_string(id);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  bool transformed;
  do
  {
    transformed = tfBuffer.canTransform("map", tag_name, ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << "map" << " to " << tag_name;
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ROS_INFO("Found transform for tag%i", id);

  geometry_msgs::TransformStamped transform_msg;
  transform_msg = tfBuffer.lookupTransform("map", tag_name, ros::Time(0));
  tf2::Transform map_to_tag;
  tf2::fromMsg(transform_msg.transform, map_to_tag);
  return map_to_tag;
}

void Tag::updateMapToTagTfs()
{
  for (int i = 0; i < tags.size(); i++)
  {
    tags[i].map_to_tag = findMapToTag(tags[i].id);
  }
}

void Tag::updateRelativeTransform(StampedTransform relative_transform)
{
  this->seq++;
  this->relative_transform = relative_transform;
}

StampedTransform Tag::getRelativeTransform() const
{
  return relative_transform;
}

void Tag::processDetection(tf2::Transform servo, ros::Time stamp)
{
  this->seq++;
  // TODO do transforms
  // Build transform
  tf2::Transform transform;
  tf2::Stamped<tf2::Transform> stamped_transform;
  stamped_transform.setData(transform);
  stamped_transform.stamp_ = stamp;
  this->transforms.emplace_front(stamped_transform);
  // TODO remove old transforms
  if (this->transforms.size() > this->list_size)
  {
    this->transforms.pop_back();
  }
}

StampedTransform Tag::getMostRecentTransform() const
{
  if (transforms.size() < 1)
  {
    throw std::runtime_error("No transforms available for this tag");
  }
  return transforms.front();
}

vector<StampedTransform> Tag::getTransforms() const
{
  return vector<StampedTransform>(std::begin(transforms), std::end(transforms));
}

tf2::Transform Tag::getMapToTag() const
{
  return this->map_to_tag;
}

int Tag::getID() const
{
  return id;
}

unsigned int Tag::getSeq() const
{
  return seq;
}

double Tag::getSize() const
{
  return tag_size;
}

long Tag::getTransformsSize() const
{
  return transforms.size();
}

ros::Duration Tag::getMaxDt()
{
  return max_dt;
}

std::vector<Tag> Tag::getTags()
{
  return tags;
}

bool Tag::isInitialized()
{
  return initialized;
}

uint Tag::getListSize()
{
  return list_size;
}


