#ifndef MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
#define MOTOR_ACCESS_VREP_MOTOR_ACCESS_H

#include <driver_access/driver_access.h>
#include <vrep_msgs/DriverMessage.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tuple>


namespace driver_access
{
typedef std::tuple<double, double, double> tuple3d;

class VREPDriverAccess : public DriverAccess
{
  public:
    VREPDriverAccess(const Limits &limits, ID id = ID::none, Mode mode = Mode::none);

    std_msgs::Header getHeader();

    double getPosition() override;
    double getVelocity() override;
    double getEffort() override;

  protected:
    void setDriverPosition(double position) override;
    void setDriverVelocity(double velocity) override;
    void setDriverEffort(double torque) override;

  private:
    void callback(const vrep_msgs::DriverMessageConstPtr &message);

    vrep_msgs::DriverMessage state;
    uint32_t seq;

    ros::CallbackQueuePtr queue;
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<ros::Subscriber> subscriber;
    boost::shared_ptr<ros::Publisher> publisher;
};
}

#endif //MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
