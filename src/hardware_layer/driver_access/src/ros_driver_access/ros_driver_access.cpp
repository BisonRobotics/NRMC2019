#include <ros_driver_access/ros_driver_access.h>

using namespace driver_access;

using sensor_msgs::JointState;
using std::vector;

ROSDriverAccess::ROSDriverAccess(vector<DriverAccess*> drivers)
{
  this->drivers = drivers;
  nh = new ros::NodeHandle;
  joint_state_publisher = new ros::Publisher;
  (*joint_state_publisher) = nh->advertise<sensor_msgs::JointState>("/joint_states", 2);
  joint_states.header.seq = 0;
  joint_states.header.frame_id = "map";
  for (int i = 0; i < drivers.size(); i++)
  {
    joint_states.name.push_back(name(drivers[i]->id) + "_joint");
    joint_states.position.push_back(0);
    joint_states.velocity.push_back(0);
    joint_states.effort.push_back(0);
  }
}

ROSDriverAccess::~ROSDriverAccess()
{
  joint_state_publisher->shutdown();
  nh->shutdown();
  delete joint_state_publisher;
  delete nh;
}

void ROSDriverAccess::publish()
{
  joint_states.header.seq++;
  joint_states.header.stamp = ros::Time::now();
  for (int i = 0; i < drivers.size(); i++)
  {
    joint_states.position[i] = drivers[i]->getPosition();
    joint_states.velocity[i] = drivers[i]->getVelocity();
    joint_states.effort[i]   = drivers[i]->getEffort();
  }
  joint_state_publisher->publish(joint_states);
}