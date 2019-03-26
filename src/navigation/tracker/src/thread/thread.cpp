#include <tracker/thread/thread.h>
#include <boost/timer/timer.hpp>


using namespace tracker;
using namespace stepper;

Thread::Thread(std::string name, tracker::Camera *camera, Stepper *stepper, TagsVector *tags) :
  name(name), camera(camera), nh(name), it(name), tags(tags), stepper(stepper),
  set_brightness_server(nh, "set_brightness", boost::bind(&Thread::setBrightnessCallback, this, _1), false),
  set_exposure_server(nh, "set_exposure", boost::bind(&Thread::setExposureCallback, this, _1), false)
{
  drops = 0;
  drop_count = 0;
  nh.setCallbackQueue(&callback_queue);
  get_brightness_service = nh.advertiseService("get_brightness", &Thread::getBrightnessCallback, this);
  get_exposure_service   = nh.advertiseService("get_exposure",   &Thread::getExposureCallback,   this);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 1);
  //pose_pub1 = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate1", 1);
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

  detector = new Detector(camera->getInfo(), image_msg.data.data(), this->tags);
  set_brightness_server.start();
  set_exposure_server.start();

  thread_handle = new boost::thread(boost::bind(&Thread::thread, this));
}

void Thread::thread()
{
  camera->start();
  ros::Time stamp;

  // Main loop
  boost::timer::cpu_timer total;
  boost::timer::cpu_timer actual;
  while (ros::ok())
  {
    // Get frame from camera (blocking)
    total.start();
    while (true)
    {
      try
      {
        camera->getFrame(detector->getBuffer());
        // TODO request stepper position
        break;
      }
      catch (std::runtime_error &e)
      {
        ROS_WARN("%s", e.what());
        ros::Duration(0.1).sleep();
      }
    }

    // Detect tags
    actual.start();
    Tag::clearFlags(tags);
    stamp = ros::Time::now();
    detector->detect(stamp);

    // Add stepper transform
    for (int i = 0; i < tags->size(); i++)
    {
      if ((*tags)[i].relativeTransformUpdated())
      {
        StampedTransform stepper_transform; // TODO request stepper position
        stepper_transform.stamp_ = stamp;
        (*tags)[i].addStepperTransform(stepper_transform);
      }
    }

    // Publish tfs
    for (int i = 0; i < tags->size(); i++)
    {
      if ((*tags)[i].relativeTransformUpdated())
      {
        StampedTransform transform = (*tags)[i].getMostRecentRelativeTransform();
        geometry_msgs::TransformStamped transform_msg = tf2::toMsg(transform);
        transform_msg.header.seq = (*tags)[i].getSeq();
        transform_msg.header.frame_id = camera->getName();
        transform_msg.child_frame_id = "tag" + std::to_string((*tags)[i].getID()) + "_estimate";
        tf_pub.sendTransform(transform_msg);
      }
    }

    // Publish pose estimate
    for (int i = 0; i < tags->size(); i++)
    {
      if ((*tags)[i].relativeTransformUpdated() &&
          (*tags)[i].stepperTransformUpdated())
      {
        if ((*tags)[i].getID() == 0)
        {
          geometry_msgs::PoseStamped pose_estimate = (*tags)[i].estimatePose();
          //pose_estimate.pose.position.z = 1.0;
          pose_pub.publish(pose_estimate);
        }
        else if ((*tags)[i].getID() == 1)
        {
          //geometry_msgs::PoseStamped pose_estimate = (*tags)[i].estimatePose();
          //pose_estimate.pose.position.z = 1.0;
          //pose_pub1.publish(pose_estimate);
        }
      }
    }

    // Control loop
    // TODO better tag selection
    for (int i = 0; i < tags->size(); i++)
    {
      if ((*tags)[i].getID() == 0 && (*tags)[i].relativeTransformUpdated())
      {
        tf2::Vector3 T = (*tags)[i].getMostRecentRelativeTransform().getOrigin();
        double error = std::atan2(T.getX(), T.getZ());
        //ROS_INFO("Error: %f", error);
      }
    }

    // Publish the image
   // printf("Delay: %s\n", timer.format(4).c_str());
    if (drop_count++ >= drops)
    {
      pub.publish(image_msg);
      drop_count = 0;
    }

    // Respond to callbacks
    callback_queue.callAvailable(ros::WallDuration());
    //ros::spinOnce();
    total.stop(); actual.stop();
    double total_time = total.elapsed().wall * 1.0e-9;
    double actual_time = actual.elapsed().wall * 1.0e-9;
    //printf("Rate(Hz): %7.4f, Utilized(%%): %7.4f\n", 1.0/total_time, actual_time/total_time);
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
