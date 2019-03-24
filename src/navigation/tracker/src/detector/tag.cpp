#include <tracker/detector/tag.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace tracker;

using std::vector;

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

Tag::Tag(int id, int list_size, double tag_size, ros::Duration max_dt) :
  id(id), tag_size(tag_size), list_size(list_size), max_dt(max_dt), seq(0)
{
  this->map_to_tag = findMapToTag(id);
}

void Tag::addDetection(cv::Point2d center, tf2::Transform camera_to_tag,
                       tf2::Transform servo, ros::Time stamp)
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

StampedTransform Tag::getMostRecentTransform()
{
  if (transforms.size() < 1)
  {
    throw std::runtime_error("No transforms available for this tag");
  }
  return transforms.front();
}

vector<StampedTransform> Tag::getTransforms()
{
  return vector<StampedTransform>(std::begin(transforms), std::end(transforms));
}

tf2::Transform Tag::getMapToTag()
{
  return this->map_to_tag;
}

int Tag::getID()
{
  return id;
}

unsigned int Tag::getSeq()
{
  return seq;
}

double Tag::getSize()
{
  return tag_size;
}

long Tag::getTransformsSize()
{
  return transforms.size();
}
