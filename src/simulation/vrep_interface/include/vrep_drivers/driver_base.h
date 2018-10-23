#ifndef MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
#define MOTOR_ACCESS_VREP_MOTOR_ACCESS_H

#include <driver_access/driver_access.h>
#include <driver_access/limits.h>
#include <vrep_msgs/VREPDriverMessage.h>
#include <vrep_msgs/VREPDriverParameters.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <vrep_plugin/interface.h>


namespace vrep_interface
{

class VREPDriverBase : public driver_access::DriverAccess
{
  public:
    VREPDriverBase(SimInterface *sim_interface, driver_access::ID id);

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
    SimInterface *sim;
    const std::string joint_name, link_name;
    int joint_handle, link_handle;
    vrep_msgs::VREPDriverMessage state;
    ros::Publisher *publisher;

  private:
    vrep_msgs::VREPDriverMessage command;
    uint32_t seq;

    ros::NodeHandle *nh;
    ros::Subscriber *subscriber;
    ros::ServiceServer *params_server;

    void commandCallback(const vrep_msgs::VREPDriverMessageConstPtr &message);
};
}

#endif //MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
