#ifndef MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
#define MOTOR_ACCESS_VREP_MOTOR_ACCESS_H

#include <driver_access/driver_access_base.h>
#include <driver_access/limits.h>
#include <vrep_msgs/VREPDriverMessage.h>

#include <vrep_library/v_repLib.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>


namespace vrep_interface
{

static const driver_access::Limits vrep_driver_limits(0, 1e10, 0, 1e10, -1e10, 1e10);

class VREPDriver : public driver_access::DriverAccessBase
{
  public:
    VREPDriver(uint8_t id, const std::string &joint_name);

    double getPosition() override;
    double getVelocity() override;
    double getEffort() override;

    std_msgs::Header getHeader();
    uint8_t getMode();
    void updateHandle();

    virtual void updateState() = 0;

  protected:
    void setDriverPosition(double position) = 0;
    void setDriverVelocity(double velocity) = 0;
    void setDriverEffort(double effort) = 0;


    vrep_msgs::VREPDriverMessage state;
    simInt handle;
    std::string joint_name;
    boost::shared_ptr<ros::Publisher> publisher;

  private:
    void callback(const vrep_msgs::VREPDriverMessageConstPtr &message);

    vrep_msgs::VREPDriverMessage command;
    uint32_t seq;

    ros::CallbackQueuePtr queue;
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<ros::Subscriber> subscriber;

};
}

#endif //MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
