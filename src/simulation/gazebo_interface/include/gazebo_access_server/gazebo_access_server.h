#ifndef GAZEBO_INTERFACE_GAZEBO_DRIVER_BASE_H
#define GAZEBO_INTERFACE_GAZEBO_DRIVER_BASE_H

#include <driver_access/driver_access.h>
#include <driver_access/limits.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{

class GazeboDriver : public driver_access::DriverAccess
{
  public:
    GazeboDriver(physics::ModelPtr model, driver_access::ID id);

    // This class is kind of a hack, so we override these
    double getPosition() override;
    double getVelocity() override;
    double getEffort() override;

    driver_access::Mode getMode() override;

    double setPoint();


  protected:
    const std::string joint_name;
    physics::ModelPtr model;
    physics::JointPtr joint;
    common::PID pid;

    void cmd_callback(ConstVector3dPtr &message);
    void setDriverPosition(double position);
    void setDriverVelocity(double velocity);
    void setDriverEffort(double effort);

  private:
    uint32_t seq;

    transport::NodePtr node;
    transport::SubscriberPtr subscriber;
};
}

#endif //GAZEBO_INTERFACE_GAZEBO_DRIVER_BASE_H
