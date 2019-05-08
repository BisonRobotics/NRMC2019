#include <tracker/thread.h>
#include <boost/timer/timer.hpp>
#include <tracker/camera/ocam_camera.h>
#include <tracker/Debug.h>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <boost/algorithm/clamp.hpp>


// Must be included once in the project
#include <tracker/tag_config.h>


using namespace tracker;
using namespace stepper;
using boost::math::double_constants::pi;
using boost::algorithm::clamp;



tracker::Camera* initializeOCam(CameraInfo info, uint brightness, uint exposure)
{
  if (exposure > 195)
  {
    exposure = 195;
    ROS_WARN("Exposure can't be greater than 195 to operate at 50Hz");
  }
  tracker::Camera* camera = nullptr;
  while (camera == nullptr)
  {
    if (!ros::ok())
    {
      ROS_WARN("ROS is not ok");
      return nullptr;
    }
    try
    {
      // brightness (int)    : min=0 max=255 step=1 default=64 value=50
      // exposure_auto (menu)   : min=0 max=3 default=1 value=1
      // exposure_absolute (int)    : min=1 max=1000 step=1 default=55 value=90 (195ish for 50Hz)
      camera = new tracker::OCamCamera(info, 50, brightness, exposure);
    }
    catch (std::runtime_error &e)
    {
      ROS_WARN("%s", e.what());
      ros::Duration(2).sleep();
    }
  }
  return camera;
}

Thread::Thread(ros::NodeHandle base_nh, ros::NodeHandle nh, Config config, CameraInfo camera_info) :
  base_nh(base_nh), nh(nh), config(config), it(nh), tf_listener(tf_buffer), active_id(1),
  set_brightness_server(nh, "set_brightness", boost::bind(&Thread::setBrightnessCallback, this, _1), false),
  set_exposure_server(nh, "set_exposure", boost::bind(&Thread::setExposureCallback, this, _1), false)
{
  Tag::init(50, ros::Duration(2.0), true);
  tags = Tag::getTags();
  Tag::setTransformCaches(&tags, camera_info.name);

  camera = initializeOCam(camera_info, (uint)config.brightness(), (uint)config.exposure());
  if (camera == nullptr)
  {
    ROS_WARN("Camera %s handle invalid, shutting down", config.name().c_str());
    return;
  }

  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

  drops = 0;
  drop_count = 0;
  nh.setCallbackQueue(&callback_queue);
  get_brightness_service = nh.advertiseService("get_brightness", &Thread::getBrightnessCallback, this);
  get_exposure_service   = nh.advertiseService("get_exposure",   &Thread::getExposureCallback,   this);

  pose_pub = base_nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 1);
  debug_pub = nh.advertise<tracker::Debug>("debug", 1);
  pub = it.advertise("image", 1);

  image_msg.header.frame_id = config.name();
  image_msg.header.seq = 0;
  image_msg.header.stamp = ros::Time::now();
  image_msg.height = camera->getHeight();
  image_msg.width = camera->getWidth();
  image_msg.encoding = "mono8";
  image_msg.is_bigendian = 0;
  image_msg.step = camera->getWidth();
  image_msg.data.resize(camera->getWidth() * camera->getHeight());

  detector = new Detector(camera->getInfo(), image_msg.data.data(), &(this->tags));
  callback_queue.enable();
  set_brightness_server.start();
  set_exposure_server.start();

  ROS_INFO("Starting thread for %s", config.name().c_str());
  thread_handle = new boost::thread(boost::bind(&Thread::thread, this));
}

void Thread::thread()
{
  ROS_INFO("Starting %s", camera->getName().c_str());
  try
  {
    camera->start();
  }
  catch (std::runtime_error &e)
  {
    ROS_WARN("[%s]: %s", camera->getName().c_str(), e.what());
  }

  ros::Time stamp;

  ROS_INFO("[%s]: Initializing stepper for %s", camera->getName().c_str(), config.name().c_str());
  stepper = new Stepper("tracker_can", (uint)config.stepperControllerID(), (uint)config.stepperClientID());
  stepper->setMode(Mode::Initialize, (float)config.maxInitializationVelocity());
  ros::Duration(5.0).sleep();
  stepper->setMode(Mode::Velocity, 0.0);

  //ROS_INFO("Attempting to set brightness = %i", config.brightness());
  //camera->setBrightness((uint)config.brightness());
  //os::Duration(1.0).sleep();
 // ROS_INFO("Brightness = %i", camera->getBrightness());
  //ros::Duration(1.0).sleep();
  //camera->setExposure(config.exposure());

  // Main loop
  boost::timer::cpu_timer total;
  boost::timer::cpu_timer actual;
  while (ros::ok())
  {
    // Get frame from camera (blocking)
    Debug debug_msg;
    total.start();
    uint i = 0;
    while (true)
    {
      if (!ros::ok()) return;
      try
      {
        camera->getFrame(detector->getBuffer());
        break;
      }
      catch (std::runtime_error &e)
      {
        ROS_WARN("[%s]: %s", camera->getName().c_str(), e.what());
        try
        {
          camera->stop();
          ros::Duration(0.5).sleep();
          camera->start();
        }
        catch (std::runtime_error &e)
        {
          ROS_WARN("[%s]: %s", camera->getName().c_str(), e.what());
        }
        if (i++ >= 4)
        {
          i = 0;
          try
          {
            ROS_INFO("[%s]: Attempting to reboot...", camera->getName().c_str());
            camera->reboot();
            ros::Duration(1.0).sleep();
            ROS_INFO("[%s]: Attempting to start...", camera->getName().c_str());
            camera->start();
          }
          catch (std::runtime_error &e)
          {
            ROS_WARN("[%s]: %s", camera->getName().c_str(), e.what());
            ros::Duration(0.1).sleep();
          }
        }
      }
    }
    //ROS_INFO("Brightness = %i", camera->getBrightness());

    // Get latest TF and select active tag
    try
    {
      tf2::fromMsg(tf_buffer.lookupTransform("map", "base_link", ros::Time(0)), transform);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
    }
    if (transform.getOrigin().y() < 0.1 && transform.getOrigin().x() < 0.1) // Don't know where we are
    {
      active_id = -1;
    }
    else if (transform.getOrigin().y() < config.tagSwitchY())
    {
      if (transform.getOrigin().x() > config.tagSwitchX())
      {
        active_id = 4; // Medium tag
      }
      else
      {
        active_id = 8; // Small tag
      }
    }
    else
    {
      active_id = 1; // Big tag
    }

      // Detect tags
    actual.start();
    Tag::clearFlags(&tags);
    stamp = ros::Time::now();
    detector->detect(stamp);

    // Add stepper transform
    State state;
    try
    {
      state = stepper->pollState();
    }
    catch (std::runtime_error &e)
    {
      ROS_ERROR("%s", e.what());
      continue;
    }

    double position = state.position;
    position = position * 2 * pi;
    StampedTransform stepper_transform;
    stepper_transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    stepper_transform.setRotation(tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), position));
    stepper_transform.stamp_ = stamp;
    geometry_msgs::TransformStamped transform_msg = tf2::toMsg(stepper_transform);
    transform_msg.header.seq = stepper->getSeq();
    transform_msg.header.frame_id = camera->getName() + "_camera_base";
    transform_msg.child_frame_id = camera->getName() + "_camera";
    tf_pub.sendTransform(transform_msg);
    for (int i = 0; i < tags.size(); i++)
    {
      if (tags[i].relativeTransformUpdated())
      {
        transform_msg.header.seq = tags[i].getSeq();
        tags[i].addStepperTransform(stepper_transform);
      }
    }

    // Publish tfs
    for (int i = 0; i < tags.size(); i++)
    {
      if (tags[i].relativeTransformUpdated())
      {
        StampedTransform transform = tags[i].getMostRecentRelativeTransform();
        geometry_msgs::TransformStamped transform_msg = tf2::toMsg(transform);
        transform_msg.header.seq = tags[i].getSeq();
        transform_msg.header.frame_id = camera->getName() + "_camera";
        transform_msg.child_frame_id = "tag" + std::to_string(tags[i].getID()) + "_" + config.name() + "_estimate";
        tf_pub.sendTransform(transform_msg);

      }
    }

    // Publish pose estimate
    for (int i = 0; i < tags.size(); i++)
    {
      if (tags[i].relativeTransformUpdated() && tags[i].stepperTransformUpdated())
      {
        geometry_msgs::PoseStamped pose_estimate = tags[i].estimatePose();
        pose_pub.publish(pose_estimate);
        break;
      }
    }

    // Control loop
    // TODO better tag selection
    bool success = false;
    for (int i = 0; i < tags.size(); i++)
    {
      if (tags[i].getID() == active_id)
      {
        try
        {
          tf2::Vector3 T = tags[i].getMostRecentRelativeTransform().getOrigin();
          double error = std::atan2(T.getX(), T.getZ()) / (2*pi);
          debug_msg.angle_error = error;
          if (std::abs(error) < 0.002) error = 0.0;
          //ROS_INFO("Error: %f", error);
          float setpoint = (float)clamp(config.k()*error, -config.maxVelocity(), config.maxVelocity());
          if (tags[i].relativeTransformUpdated())
          {
            drop_count = 0;
            stepper->setMode(Mode::Velocity, setpoint);
          }
          else if (drop_count <= 50)
          {
            stepper->setMode(Mode::Velocity, setpoint);
          }
          else
          {
            //double dir = (error >= 0 ? 1.0 : -1.0);
            //double value = std::max(-0.02, std::min(dir*0.001*drop_count, 0.02)); // Clamp
            //stepper->setMode(Mode::Scan, 0.5*dir*gain);
          }
        }
        catch (std::runtime_error &e)
        {
          //printf("No recent transforms\n");
        }
      }
    }
    if (drop_count++ > 50)
    {
      if (active_id == -1)
      {
        stepper->setMode(Mode::Scan, (float)config.initialScanVelocity());
      }
      else
      {
        stepper->setMode(Mode::Scan, (float)config.maxScanVelocity());
      }
    }


    // Publish the image
    pub.publish(image_msg);

    // Respond to callbacks
    ros::spinOnce();
    callback_queue.callAvailable(ros::WallDuration());

    // Output debug info
    total.stop(); actual.stop();
    double total_time = total.elapsed().wall * 1.0e-9;
    double actual_time = actual.elapsed().wall * 1.0e-9;
    debug_msg.rate = 1.0/total_time;
    debug_msg.cpu_utilization = actual_time/total_time;
    debug_pub.publish(debug_msg);
    //printf("Rate(Hz): %7.4f, Utilized(%%): %7.4f\n", 1.0/total_time, actual_time/total_time);
  }

  // Exit
  stepper->setMode(Mode::Disabled, 0.0f);
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
    uint32_t exposure = goal->value;
    if (exposure > 195)
    {
      exposure = 195;
      ROS_WARN("Exposure can't be greater than 195 to operate at 50Hz");
    }
    camera->setExposure(exposure);
    set_exposure_server.setSucceeded();
  }
  catch (std::runtime_error &e)
  {
    ROS_ERROR("%s", e.what());
    set_exposure_server.setAborted();
  }
}
