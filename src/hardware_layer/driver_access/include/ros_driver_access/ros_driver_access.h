#ifndef DRIVER_ACCESS_ROS_DRIVER_ACCESS_H
#define DRIVER_ACCESS_ROS_DRIVER_ACCESS_H

#include <driver_access/driver_access.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace driver_access
{
class ROSDriverAccess
{
  public:
    explicit ROSDriverAccess(std::vector<DriverAccess*> drivers);
    ~ROSDriverAccess();
    void publish();

  private:
    unsigned int seq;
    sensor_msgs::JointState joint_states;

    std::vector<DriverAccess*> drivers;
    ros::NodeHandle *nh;
    ros::Publisher *joint_state_publisher;
};
}

#endif //DRIVER_ACCESS_ROS_DRIVER_ACCESS_H
