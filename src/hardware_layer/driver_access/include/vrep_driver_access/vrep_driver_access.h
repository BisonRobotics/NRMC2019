#ifndef MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
#define MOTOR_ACCESS_VREP_MOTOR_ACCESS_H

#include <driver_access/driver_access_base.h>
#include <vrep_msgs/VREPDriverMessage.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>


namespace driver_access
{
class VREPDriverAccess : public DriverAccessBase
{
  public:
    VREPDriverAccess(const Limits &limits, uint8_t id);

    std_msgs::Header getHeader();

    double getVelocity() override;
    double getTorque() override;
    double getPosition() override;

  protected:
    void setDriverVelocity(double velocity) override;
    void setDriverTorque(double torque) override;
    void setDriverPosition(double position) override;

  private:
    void callback(const vrep_msgs::VREPDriverMessageConstPtr &message);

    vrep_msgs::VREPDriverMessage current;
    uint32_t seq;

    ros::CallbackQueuePtr queue;
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<ros::Subscriber> subscriber;
    boost::shared_ptr<ros::Publisher> publisher;

};
}

#endif //MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
