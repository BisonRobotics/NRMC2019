#include <vrep_drivers/vrep_wheel_driver.h>

using namespace vrep_interface;

using driver_access::Limits;
using driver_access::ID;
using driver_access::name;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;

VREPWheelDriver::VREPWheelDriver(SimInterface *sim_interface, ID id) :
    VREPDriverBase(sim_interface, id) {}

void VREPWheelDriver::updateState()
{
  // TODO add conversions
  state.position = sim->getPosition(handle);
  state.velocity = sim->getVelocity(handle) * 0.1524;
  state.effort = sim->getEffort(handle);
  updateHeader(&state.header);
  publisher->publish(state);
}

void VREPWheelDriver::setDriverPosition(double position)
{
  sim->setJointPosition(handle, position);
}

void VREPWheelDriver::setDriverVelocity(double velocity)
{
  sim->setVelocity(handle, velocity / 0.1524);
}

void VREPWheelDriver::setDriverEffort(double effort)
{
  sim->setEffort(handle, effort);
}
