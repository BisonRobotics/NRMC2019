#ifndef TRACKER_THREAD_H
#define TRACKER_THREAD_H

#include <tracker/camera/camera.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <tracker/detector/detector.h>
#include <tracker/GetUInt.h>
#include <tracker/SetUIntAction.h>
#include <actionlib/server/simple_action_server.h>

namespace tracker
{
  class Thread
  {
  public:
    Thread(std::string name, Camera *camera);

    void thread();
    void join();
    bool getBrightnessCallback(GetUInt::Request &req, GetUInt::Response &res);
    bool getExposureCallback(GetUInt::Request &req, GetUInt::Response &res);
    void setBrightnessCallback(const actionlib::SimpleActionServer<SetUIntAction>::GoalConstPtr &goal);
    void setExposureCallback(const actionlib::SimpleActionServer<SetUIntAction>::GoalConstPtr &goal);

  private:
    int drops, drop_count;
    std::string name;
    ros::NodeHandle nh;
    ros::ServiceServer get_brightness_service, get_exposure_service;
    actionlib::SimpleActionServer<SetUIntAction> set_brightness_server, set_exposure_server;
    image_transport::ImageTransport it;
    image_transport::Publisher pub;
    sensor_msgs::Image image_msg;
    ros::CallbackQueue callback_queue;
    Camera *camera;
    Detector *detector;
    boost::thread *thread_handle;
  };
}

#endif //TRACKER_THREAD_H
