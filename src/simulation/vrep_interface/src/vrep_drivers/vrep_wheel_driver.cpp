#include <vrep_drivers/vrep_wheel_driver.h>

using namespace vrep_interface;

using std::get;
using driver_access::Limits;
using driver_access::ID;
using driver_access::name;
using vrep_msgs::VREPDriverMessage;
using vrep_msgs::VREPDriverMessageConstPtr;

VREPWheelDriver::VREPWheelDriver(SimInterface *sim_interface, ID id) :
    VREPDriverBase(sim_interface, id)
{
  radius = 0.0;
}

void VREPWheelDriver::updateState()
{
  // TODO add conversions
  state.position = sim->getPosition(joint_handle);
  state.velocity = sim->getVelocity(joint_handle) * radius;
  state.effort = sim->getEffort(joint_handle);
  updateHeader(&state.header);
  publisher->publish(state);
}

void VREPWheelDriver::setDriverPosition(double position)
{
  sim->setJointPosition(joint_handle, position);
}

void VREPWheelDriver::setDriverVelocity(double velocity)
{
  sim->setVelocity(joint_handle, velocity / radius);
}

void VREPWheelDriver::setDriverEffort(double effort)
{
  sim->setEffort(joint_handle, effort);
}

void VREPWheelDriver::initializeChild()
{
  radius = get<0>(sim->getObjectSize(link_handle)) / 2.0;
}
