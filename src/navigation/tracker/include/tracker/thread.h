#ifndef TRACKER_THREAD_H
#define TRACKER_THREAD_H

#include <tracker/camera/camera.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <tracker/detector.h>
#include <tracker/GetUInt.h>
#include <tracker/SetUIntAction.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stepper/stepper.h>
#include <tracker/config.h>

namespace tracker
{
  class Thread
  {
  public:
    Thread(ros::NodeHandle base_nh, ros::NodeHandle nh, Config config, CameraInfo camera_info);

    void thread();
    void join();
    bool getBrightnessCallback(GetUInt::Request &req, GetUInt::Response &res);
    bool getExposureCallback(GetUInt::Request &req, GetUInt::Response &res);
    void setBrightnessCallback(const actionlib::SimpleActionServer<SetUIntAction>::GoalConstPtr &goal);
    void setExposureCallback(const actionlib::SimpleActionServer<SetUIntAction>::GoalConstPtr &goal);

  private:
    int drops, drop_count;
    sensor_msgs::Image image_msg;
    Camera *camera;
    stepper::Stepper *stepper;
    Detector *detector;
    boost::thread *thread_handle;
    std::vector<Tag> tags;
    Config config;
    int active_id;

    ros::NodeHandle base_nh, nh;
    ros::Publisher pose_pub, pose_pub1, debug_pub;
    ros::ServiceServer get_brightness_service, get_exposure_service;
    actionlib::SimpleActionServer<SetUIntAction> set_brightness_server, set_exposure_server;
    image_transport::ImageTransport it;
    image_transport::Publisher pub;
    ros::CallbackQueue callback_queue;
    tf2_ros::TransformBroadcaster tf_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2::Stamped<tf2::Transform> transform;
  };
}

#endif //TRACKER_THREAD_H
