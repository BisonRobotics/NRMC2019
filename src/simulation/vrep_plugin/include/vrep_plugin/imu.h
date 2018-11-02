#ifndef VREP_PLUGIN_IMU_H
#define VREP_PLUGIN_IMU_H

#include <vrep_plugin/interface.h>
#include <ros/ros.h>

namespace vrep_plugin
{

class IMU
{
  public:
    IMU(Interface *sim_interface, uint8_t id);

    void updateState();
    void updateHeader(std_msgs::Header *header);


  private:
    unsigned int seq;
    tuple3d angular_velocity;
    tuple3d linear_acceleration;
    ros::NodeHandle *nh;
    ros::Publisher *publisher;

};

}

#endif //VREP_PLUGIN_IMU_H
