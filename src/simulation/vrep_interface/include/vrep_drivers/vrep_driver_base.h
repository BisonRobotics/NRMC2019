#ifndef MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
#define MOTOR_ACCESS_VREP_MOTOR_ACCESS_H

#include <driver_access/driver_access.h>
#include <driver_access/limits.h>
#include <vrep_msgs/VREPDriverMessage.h>
#include <vrep_msgs/PIDGet.h>
#include <vrep_msgs/PIDSet.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <vrep_interface/sim_interface.h>


namespace vrep_interface
{

class VREPDriverBase : public driver_access::DriverAccess
{
  public:
    VREPDriverBase(SimInterface *sim_interface, driver_access::ID id);

    // This class is kind of a hack, so we override these
    double getPosition() override;
    double getVelocity() override;
    double getEffort() override;
    driver_access::Mode getMode() override;

    double setPoint();
    void updateHeader(std_msgs::Header *header);
    void updateHandle();
    void shutdown();

    virtual void updateState() = 0;

  protected:
    SimInterface *sim;
    const std::string joint_name;
    vrep_msgs::VREPDriverMessage state;
    int handle;
    ros::Publisher *publisher;

  private:
    vrep_msgs::VREPDriverMessage command;
    uint32_t seq;

    ros::NodeHandle *nh;
    ros::Subscriber *subscriber;
    ros::ServiceServer *pid_set_server;
    ros::ServiceServer *pid_get_server;

    void callback(const vrep_msgs::VREPDriverMessageConstPtr &message);
    bool getPIDCallback(vrep_msgs::PIDGetRequest &req, vrep_msgs::PIDGetResponse &res);
    bool setPIDCallback(vrep_msgs::PIDSetRequest &req, vrep_msgs::PIDSetResponse &res);
};
}

#endif //MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
