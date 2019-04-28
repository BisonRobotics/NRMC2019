#include <tracker/tag.h>
#include <Eigen/Dense>

/*
 * Comparison operators for unit tests
 */
namespace tracker
{
bool operator == (const Tag &lhs, const Tag &rhs)
{
  return lhs.getID()         == rhs.getID()         &&
         lhs.getSeq()        == rhs.getSeq()        &&
         lhs.getSize()       == rhs.getSize();
}

bool operator != (const Tag &lhs, const Tag &rhs)
{
  return !(lhs.getID()         == rhs.getID()         &&
           lhs.getSeq()        == rhs.getSeq()        &&
           lhs.getSize()       == rhs.getSize());
}
}

/*
 * Types
 */
using namespace tracker;
using std::vector;

/*
 * Constructors
 */
Tag::Tag(const Tag &tag) : id(tag.id), seq(tag.seq), tag_size(tag.tag_size), tag_to_map(tag.tag_to_map){}
Tag::Tag(Tag *tag) : id(tag->id), seq(tag->seq), tag_size(tag->tag_size), tag_to_map(tag->tag_to_map){}
Tag::Tag(int id, double tag_size) : id(id), tag_size(tag_size), seq(0)
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

/*
 * Methods
 */
void Tag::addRelativeTransform(StampedTransform relative_transform)
{
  addTransformToList(&relative_transforms, relative_transform);
  relative_transform_updated = true;
}

void Tag::addStepperTransform(StampedTransform stepper_transform)
{
  addTransformToList(&stepper_transforms, stepper_transform);
  this->stepper_transform_updated = true;
}

geometry_msgs::PoseStamped Tag::estimatePose()
{
  if (!(relative_transform_updated && stepper_transform_updated))
  {
    throw std::runtime_error("One of the transforms was not updated");
  }

  StampedTransform stepper_transform = stepper_transforms.front();
  StampedTransform relative_transform = relative_transforms.front();

  tf2::Transform base_link_to_map = base_link_to_camera_base
                                    * stepper_transform
                                    * relative_transform
                                    * tag_to_map;

  //tf2::Vector3 O = base_link_to_map.getOrigin();
  //printf("%f, %f, %f\n", O.getX(), O.getY(), O.getZ());

  /*tf2::Transform T1, T2, T3;
  T1.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  T1.setRotation(base_link_to_map.getRotation().inverse());
  T2.setOrigin(-base_link_to_map.getOrigin());
  T2.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  T3 = T1 * T2;*/

  geometry_msgs::PoseStamped pose_estimate;
  tf2::toMsg(base_link_to_map.inverse(), pose_estimate.pose);
  pose_estimate.header.seq = this->seq++;
  pose_estimate.header.frame_id = "map";
  pose_estimate.header.stamp = stepper_transforms.front().stamp_;

  return pose_estimate;
}

/*
 * Accessors
 */
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
  return relative_transforms.size();
}

tf2::Transform Tag::getMapToTag() const
{
  return this->tag_to_map;
}

StampedTransform Tag::getMostRecentRelativeTransform() const
{
  if (relative_transforms.empty())
  {
    throw std::runtime_error("No transforms available for this tag");
  }
  return relative_transforms.front();
}

vector<StampedTransform> Tag::getRelativeTransforms() const
{
  return vector<StampedTransform>(std::begin(relative_transforms), std::end(relative_transforms));
}

bool Tag::relativeTransformUpdated() const
{
  return relative_transform_updated;
}

bool Tag::stepperTransformUpdated() const
{
  return stepper_transform_updated;
}

/*
 * Static members
 */
std::vector<Tag> Tag::tags;
bool Tag::initialized = false;
uint Tag::list_size = 0;
ros::Duration Tag::max_dt;

/*
 * Static methods
 */
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

std::vector<Tag> Tag::getTags()
{
  return tags;
}

uint Tag::getListSize()
{
  return list_size;
}

ros::Duration Tag::getMaxDt()
{
  return max_dt;
}

bool Tag::isInitialized()
{
  return initialized;
}


tf2::Transform Tag::findMapToTag(int id)
{
  std::string tag_name = "tag" + std::to_string(id);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  do
  {
    if (tfBuffer.canTransform(tag_name, "map", ros::Time(0), ros::Duration(1)))
    {
      break;
    }
    else
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << tag_name << " to " << "map";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok());
  ROS_INFO("Found transform for tag%i", id);

  geometry_msgs::TransformStamped transform_msg;
  transform_msg = tfBuffer.lookupTransform(tag_name, "map", ros::Time(0));
  tf2::Transform tag_to_map;
  tf2::fromMsg(transform_msg.transform, tag_to_map);
  return tag_to_map;
}

void Tag::updateMapToTagTfs()
{
  for (int i = 0; i < tags.size(); i++)
  {
    tags[i].tag_to_map = findMapToTag(tags[i].id);
  }
}

void Tag::setTransformCaches(TagsVector *tags, std::string prefix)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Find base link to tracker mount transform
  std::string camera_base = prefix + "_camera_base";
  do
  {
    if (tfBuffer.canTransform("base_link", camera_base, ros::Time(0), ros::Duration(1)))
    {
      break;
    }
    else
    {
      ROS_WARN("Unable to find transform from map to %s", camera_base.c_str());
    }
  } while(ros::ok());
  ROS_INFO("Found transform from map to %s", camera_base.c_str());

  StampedTransform base_link_to_camera_base;
  tf2::fromMsg(tfBuffer.lookupTransform("base_link", camera_base, ros::Time(0)), base_link_to_camera_base);

  for (int i = 0; i < tags->size(); i++)
  {
    (*tags)[i].base_link_to_camera_base = (tf2::Transform)base_link_to_camera_base;
  }
}

void Tag::clearFlags(TagsVector *tags)
{
  for (int i = 0; i < tags->size(); i++)
  {
    (*tags)[i].relative_transform_updated = false;
    (*tags)[i].stepper_transform_updated = false;
  }
}

void Tag::addTransformToList(StampedTransformList *list, const StampedTransform &transform)
{
  // Clear expired transforms
  // TODO Test expired transform removal
  auto expired_start = list->end();
  for (auto it = list->begin(); it != list->end(); it++)
  {
    if ((transform.stamp_ - (*it).stamp_) > Tag::max_dt)
    {
      expired_start = it;
      break;
    }
  }
  list->erase(expired_start, list->end());

  // Add new transform
  list->emplace_front(transform);
  if (list->size() > Tag::list_size)
  {
    list->pop_back();
  }
}

uint Tag::getRelativeTransformsSize() const
{
  return relative_transforms.size();
}

