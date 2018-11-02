#include <vrep_plugin/imu.h>
#include <std_msgs/Header.h>

using namespace vrep_plugin;

void IMU::updateHeader(std_msgs::Header *header)
{
  header->stamp = ros::Time::now();
  header->seq = seq++;
}