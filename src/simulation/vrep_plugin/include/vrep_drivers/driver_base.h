#ifndef VREP_PLUGIN_DRIVER_BASE_H
#define VREP_PLUGIN_DRIVER_BASE_H

#include <driver_access/driver_access.h>
#include <driver_access/limits.h>
#include <vrep_msgs/DriverMessage.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <vrep_plugin/interface.h>


namespace vrep_plugin
{

class DriverBase : public driver_access::DriverAccess
{
  public:
    DriverBase(Interface *sim_interface, driver_access::ID id);

    double getPosition() override;
    double getVelocity() override;
    double getEffort() override;
    driver_access::Mode getMode() override;

    double setPoint();
    void updateHeader(std_msgs::Header *header);
    void initialize();
    void shutdown();

    virtual void updateState() = 0;
    virtual void initializeChild() = 0;

  protected:
    Interface *sim;
    const std::string joint_name, link_name;
    int joint_handle, link_handle;
    vrep_msgs::DriverMessage state;
    ros::Publisher *publisher;

  private:
    vrep_msgs::DriverMessage command;
    uint32_t seq;

    ros::NodeHandle *nh;
    ros::Subscriber *subscriber;
    ros::ServiceServer *params_server;

    void commandCallback(const vrep_msgs::DriverMessageConstPtr &message);
};
}

#endif //VREP_PLUGIN_DRIVER_BASE_H
