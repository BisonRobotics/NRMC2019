#include <tracker/detector/tag.h>

using namespace tracker;

Tag::Tag(int id, int priority, double size)
{
  this->id = id;
  this->priority = priority;
  this->size = size;
  this->mutex = new boost::mutex;
  this->seq = 0;
  this->list_size = 20;
  this->compare_mode = CompareType::distance;
}

void Tag::addTransform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf,
                       tf2::Stamped<tf2::Transform> servo_tf, unsigned int seq)
{
  mutex->lock();
  this->seq = seq;
  Transform new_transform(detection, tag_tf, servo_tf, this->map_to_tag_tf, &(this->compare_mode));
  this->transforms.emplace_front(new_transform);
  if (this->transforms.size() > this->list_size)
  {
    this->transforms.pop_back();
  }
  mutex->unlock();
}

std::vector<Transform> Tag::getTransforms()
{
  return std::vector<Transform>(std::begin(transforms), std::end(transforms));
}

Transform Tag::getMostRecentTransform()
{
  if (transforms.size() < 1)
  {
    throw unable_to_find_transform_error("No transforms available for this tag");
  }
  return transforms.front();
}

Transform Tag::getMedianFilteredTransform()
{
  this->compare_mode = CompareType::theta;
  if (transforms.size() < 5)
  {
    throw std::runtime_error("Median filter not populated");
  }
  mutex->lock();
  std::list<Transform> transforms_copy(transforms.begin(), transforms.end());
  transforms_copy.sort();
  mutex->unlock();
  auto it = transforms_copy.begin();
  for (int i = 0; i < (transforms_copy.size() / 2); i++)
  {
    it++;
  }
  return *it;
}

void Tag::flushOldTransforms(std::list<Transform> *transforms, ros::Time current_time, ros::Duration max_dt)
{
  auto it = transforms->begin();
  int good_tfs = 0;
  bool clean = false;
  for (int i = 0; i < transforms->size(); i++, it++)
  {
    ros::Duration time_diff(current_time - it->getTagTf().stamp_);
    if (time_diff > max_dt)
    {
      clean = true;
      good_tfs = i;
      break;
    }
  }
  if (clean)
  {
    for (int i = 0; (i + good_tfs) < transforms->size(); i++)
    {
      transforms->pop_back();
    }
  }
}

void Tag::getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 matrix;
  matrix.setRotation(q);
  matrix.getRPY(roll, pitch, yaw);
}

double Tag::getTheta(tf2::Quaternion orientation)
{
  double roll, pitch, yaw;
  getRPY(orientation, roll, pitch, yaw);
  return pitch;
}

// Assumes that the thetas are close enough not to negate each other
tf2::Quaternion Tag::getAverageOrientation(std::list<Transform> &transforms, int filter_size)
{
  double x = 0.0, y = 0.0;
  int i = 0;
  for (auto it = transforms.begin(); i < filter_size && it != transforms.end(); i++, it++)
  {
    tf2::Quaternion orientation = it->getTagTf().getRotation();
    x += cos(getTheta(orientation));
    y += sin(getTheta(orientation));
  }
  tf2::Quaternion start_orientation, corrected_orientation;
  start_orientation.setRPY(0.0, M_PI, M_PI);
  corrected_orientation.setRPY(0.0, -atan2(y,x), 0.0);
  return start_orientation*corrected_orientation;
}

tf2::Vector3 Tag::getAveragePosition(std::list<Transform> &transforms, int filter_size)
{
  double x = 0.0, y = 0.0, z = 0.0;
  int i = 0;
  for (auto it = transforms.begin(); i < filter_size && it != transforms.end(); i++, it++)
  {
    tf2::Vector3 position = it->getTagTf().getOrigin();
    x += position.x();
    y += position.y();
    z += position.z();
  }
  tf2::Vector3 position(x / filter_size, y / filter_size, z / filter_size);

  return position;
}

Transform Tag::getMovingAverageTransform(int n_tf, double max_dt)
{
  mutex->lock();

  try
  {
    flushOldTransforms(&transforms, ros::Time::now(), ros::Duration(max_dt));
    if (transforms.size() < n_tf)
    {
      throw std::runtime_error("Moving average filter not populated");
    }

    this->compare_mode = CompareType::distance;

    Transform transform = transforms.front();
    TagDetection detection = transform.getDetection();
    tf2::Transform map_to_tag_tf = transform.getMapToTagTf();
    tf2::Stamped<tf2::Transform> tag_tf = transform.getTagTf();
    tf2::Stamped<tf2::Transform> servo_tf = transform.getServoTf();
    tf2::Transform average_transform(getAverageOrientation(transforms, n_tf), getAveragePosition(transforms, n_tf));
    tf2::Stamped<tf2::Transform> stamped_transform(average_transform, ros::Time::now(), tag_tf.frame_id_);
    Transform new_transform(detection, stamped_transform, servo_tf, map_to_tag_tf, &(this->compare_mode));

    mutex->unlock();

    return new_transform;
  }
  catch (std::exception &e)
  {
    mutex->unlock();
    throw e;
  }
}

Transform Tag::getMovingAverageTransform(int n_tf)
{
  return Tag::getMovingAverageTransform(n_tf, 1.0);
}

Transform Tag::getMovingAverageTransform()
{
  return Tag::getMovingAverageTransform(this->list_size);
}

Transform Tag::getMedianMovingAverageTransform()
{
  if (transforms.size() < list_size)
  {
    throw std::runtime_error("Moving average filter not populated");
  }
  mutex->lock();
  std::list<Transform> transforms_copy(transforms.begin(), transforms.end());
  mutex->unlock();
  transforms_copy.sort();
  // Don't look at first or last two worst transforms
  auto it_begin = transforms_copy.begin();
  it_begin++;
  it_begin++;
  auto it_end = transforms_copy.end();
  it_end--;
  it_end--;
  Transform tag_transform = transforms_copy.front();
  tf2::Stamped<tf2::Transform> tag_tf = tag_transform.getTagTf();
  double x = 0, y = 0, z = 0;
  for (auto it = it_begin; it != it_end; it++)
  {
    tf2::Vector3 origin = it->getTagTf().getOrigin();
    x += origin.getX();
    y += origin.getY();
    z += origin.getZ();
  }
  double size = transforms.size();
  tag_tf.setOrigin(tf2::Vector3(x / size, y / size, z / size));
  // TODO average the Quaternions?
  return Transform(tag_transform.getDetection(), tag_tf, tag_transform.getServoTf(), tag_transform.getMapToTagTf(),
                   &(this->compare_mode));
}

double Tag::getAngleFromCenter(int n_tf)
{
  tf2::Vector3 origin = getMovingAverageTransform(n_tf).getTagTf().getOrigin();
  return atan(origin.getX() / origin.getZ());
}

double Tag::getAngleFromCenter()
{
  return getAngleFromCenter(5);
}

int Tag::getID()
{
  return id;
}

unsigned int Tag::getSeq()
{
  return seq;
}

double Tag::getGoodness()
{
  return goodness;
}

double Tag::getPriority()
{
  return priority;
}

double Tag::getSize()
{
  return size;
}

void Tag::setMapToTagTf(tf2::Transform tf) // TODO add unit test
{
  this->map_to_tag_tf = tf;
}

tf2::Transform Tag::getMapToTagTf()
{
  return this->map_to_tag_tf;
}

cv::Point Tag::getDetectionCenter()
{
  return transforms.front().getDetectionCenter();
}

long Tag::getTransformsListSize()
{
  return transforms.size();
}