#ifndef VREP_PLUGIN_IMU_H
#define VREP_PLUGIN_IMU_H

#include <vrep_plugin/interface.h>
#include <ros/ros.h>
#include <vrep_msgs/IMU.h>

namespace vrep_plugin
{

class IMU
{
  public:
    IMU(Interface *sim_interface, uint8_t id);

    void initialize();
    void updateState();
    void shutdown();
    void reset();

  private:
    bool initial_pass;
    Interface *sim;
    int sensor_handle, link_handle;
    const std::string sensor_name, link_name;
    vrep_msgs::IMU imu;
    ros::Time last_time;
    double mass;
    Eigen::Matrix<double, 4, 4> last_matrix;

    ros::NodeHandle *nh;
    ros::Publisher *publisher;
};

}

#endif //VREP_PLUGIN_IMU_H
