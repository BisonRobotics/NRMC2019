#include <tracker/thread/thread.h>

using namespace tracker;

Thread::Thread(std::string name, tracker::Camera *camera, TagsVector tags) :
  name(name), camera(camera), nh(name), it(name), tags(tags),
  set_brightness_server(nh, "set_brightness", boost::bind(&Thread::setBrightnessCallback, this, _1), false),
  set_exposure_server(nh, "set_exposure", boost::bind(&Thread::setExposureCallback, this, _1), false)
{
  drops = 0;
  drop_count = 0;
  nh.setCallbackQueue(&callback_queue);
  get_brightness_service = nh.advertiseService("get_brightness", &Thread::getBrightnessCallback, this);
  get_exposure_service   = nh.advertiseService("get_exposure",   &Thread::getExposureCallback,   this);

  pub = it.advertise("image", 1);

  image_msg.header.frame_id = name;
  image_msg.header.seq = 0;
  image_msg.header.stamp = ros::Time::now();
  image_msg.height = camera->getHeight();
  image_msg.width = camera->getWidth();
  image_msg.encoding = "mono8";
  image_msg.is_bigendian = 0;
  image_msg.step = camera->getWidth();
  image_msg.data.resize(camera->getWidth() * camera->getHeight());

  detector = new Detector(camera->getInfo(), image_msg.data.data(), &this->tags);
  set_brightness_server.start();
  set_exposure_server.start();

  thread_handle = new boost::thread(boost::bind(&Thread::thread, this));
}

void Thread::thread()
{
  camera->start();
  ros::Time stamp;

  // Main loop
  while (ros::ok())
  {
    // Get frame from camera (blocking)
    while (true)
    {
      try
      {
        camera->getFrame(detector->getBuffer());
        stamp = ros::Time::now();
        detector->detect(stamp);
        break;
      }
      catch (std::runtime_error &e)
      {
        ROS_WARN("%s", e.what());
        ros::Duration(0.1).sleep();
      }
    }

    // Publish the image
    if (drop_count++ >= drops)
    {
      pub.publish(image_msg);
      drop_count = 0;
    }

    // Publish tfs
    for (int i = 0; i < tags.size(); i++)
    {
      StampedTransform transform = tags[i].getRelativeTransform();
      if (transform.stamp_ == stamp)
      {
        geometry_msgs::TransformStamped transform_msg = tf2::toMsg(transform);
        transform_msg.header.seq = tags[i].getSeq();
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = "tag" + std::to_string(tags[i].getID());
        tf_pub.sendTransform(transform_msg);
      }
    }

    // Respond to callbacks
    callback_queue.callAvailable(ros::WallDuration());
    ros::spinOnce();
  }

  // Exit
  camera->stop();
}

void Thread::join()
{
  thread_handle->join();
}

bool Thread::getBrightnessCallback(GetUInt::Request &req, GetUInt::Response &res)
{
  try
  {
    res.value = camera->getBrightness();
  }
  catch (std::runtime_error &e)
  {
    ROS_ERROR("%s", e.what());
    return false;
  }
  return true;
}

bool Thread::getExposureCallback(GetUInt::Request &req, GetUInt::Response &res)
{
  try
  {
    res.value = camera->getExposure();
  }
  catch (std::runtime_error &e)
  {
    ROS_ERROR("%s", e.what());
    return false;
  }
  return true;}

void Thread::setBrightnessCallback(const actionlib::SimpleActionServer<SetUIntAction>::GoalConstPtr &goal)
{
  try
  {
    camera->setBrightness(goal->value);
    set_brightness_server.setSucceeded();
  }
  catch (std::runtime_error &e)
  {
    ROS_ERROR("%s", e.what());
    set_brightness_server.setAborted();
  }
}

void Thread::setExposureCallback(const actionlib::SimpleActionServer<SetUIntAction>::GoalConstPtr &goal)
{
  try
  {
    camera->setExposure(goal->value);
    set_exposure_server.setSucceeded();
  }
  catch (std::runtime_error &e)
  {
    ROS_ERROR("%s", e.what());
    set_exposure_server.setAborted();
  }
}