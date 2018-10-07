#include <vrep_drivers/vrep_wheel_driver.h>

using namespace vrep_interface;

using driver_access::Limits;
using driver_access::ID;
using driver_access::name;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;

VREPWheelDriver::VREPWheelDriver(ID id) : VREPDriverBase(id) {}

void VREPWheelDriver::updateState()
{
  // TODO add conversions
  simFloat position;
  simGetJointPosition(handle, &position);
  state.position = (double)position;

  simFloat velocity;
  simGetObjectFloatParameter(handle, 2012, &velocity);
  state.velocity = (double)(velocity * 0.1524f);

  simFloat force;
  simGetJointForce(handle, &force);
  state.effort = (double)force;

  updateHeader(&state.header);
  publisher->publish(state);
}

void VREPWheelDriver::setDriverPosition(double position)
{
  throw std::runtime_error("[setDriverPosition]: not implemented");
}

void VREPWheelDriver::setDriverVelocity(double velocity)
{
  // TODO properly scale velocity
  if(simSetJointTargetVelocity(handle, (simFloat)(velocity / 0.1524)) == -1)
  {
    throw std::runtime_error("[setDriverVelocity]: unable to set target velocity for " + joint_name);
  }
}

void VREPWheelDriver::setDriverEffort(double effort)
{
  throw std::runtime_error("[setDriverEffort]: not implemented");
}
