#ifndef MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
#define MOTOR_ACCESS_VREP_MOTOR_ACCESS_H

#include <driver_access/driver_access.h>
#include <vrep_msgs/VREPDriverMessage.h>
#include <vrep_msgs/PIDGet.h>
#include <vrep_msgs/PIDSet.h>
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

    void setPID(double p, double i, double d);
    tuple3d getPID();


  protected:
    void setDriverPosition(double position) override;
    void setDriverVelocity(double velocity) override;
    void setDriverEffort(double torque) override;

  private:
    void callback(const vrep_msgs::VREPDriverMessageConstPtr &message);

    vrep_msgs::VREPDriverMessage state;
    uint32_t seq;

    ros::CallbackQueuePtr queue;
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<ros::Subscriber> subscriber;
    boost::shared_ptr<ros::Publisher> publisher;
    boost::shared_ptr<ros::ServiceClient> pid_set_client;
    boost::shared_ptr<ros::ServiceClient> pid_get_client;
};
}

#endif //MOTOR_ACCESS_VREP_MOTOR_ACCESS_H
