#include <vrep_drivers/vrep_wheel_driver.h>
#include <driver_access/mode.h>

using namespace vrep_interface;

using std::to_string;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;

VREPWheelDriver::VREPWheelDriver(uint8_t id, const std::string &joint_name) : VREPDriver(id, joint_name)
{
  handle = -1;
}

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

  state.header = getHeader();
  publisher->publish(state);
}

void VREPWheelDriver::setDriverPosition(double position)
{
  throw std::runtime_error("[setDriverPosition] not implemented");
}

void VREPWheelDriver::setDriverVelocity(double velocity)
{
  // TODO properly scale velocity
  if(simSetJointTargetVelocity(handle, (simFloat)(velocity / 0.1524)) == -1)
  {
    throw std::runtime_error("[setDriverVelocity] unable to set target velocity for " +
                            joint_name);
  }
}

void VREPWheelDriver::setDriverEffort(double effort)
{
  throw std::runtime_error("[setDriverEffort] not implemented");
}
