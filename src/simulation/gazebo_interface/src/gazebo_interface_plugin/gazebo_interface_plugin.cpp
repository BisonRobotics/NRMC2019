#include <gazebo_interface_plugin/gazebo_interface_plugin.h>
#include <ros/ros.h>
#include <driver_access/params.h>

using namespace gazebo;

using driver_access::ID;

void DriverPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO("[DriverPlugin]: ROS is initialized");
  ROS_INFO("[DriverPlugin]: loading wheels");
  this->model = _model;
  fl = new GazeboDriver(model, ID::front_left_wheel);
  fr = new GazeboDriver(model, ID::front_right_wheel);
  br = new GazeboDriver(model, ID::back_right_wheel);
  bl = new GazeboDriver(model, ID::back_left_wheel);


}

DriverPlugin::~DriverPlugin()
{
  delete fl;
  delete fr;
  delete br;
  delete bl;
}