#include <vrep_drivers/driver_wheel.h>

using namespace vrep_plugin;

using std::get;
using driver_access::Limits;
using driver_access::ID;
using driver_access::name;

WheelDriver::WheelDriver(Interface *sim_interface, ID id) :
    DriverBase(sim_interface, id), radius(0.0) {}

void WheelDriver::updateState()
{
  //TODO double check conversions
  state.position = sim->getPosition(joint_handle) * radius;
  state.velocity = sim->getVelocity(joint_handle) * radius;
  state.effort = sim->getEffort(joint_handle) * radius;
  updateHeader(&state.header);
  publisher->publish(state);
}

void WheelDriver::setDriverPosition(double position)
{
  sim->setJointPosition(joint_handle, position / radius);
}

void WheelDriver::setDriverVelocity(double velocity)
{
  sim->setVelocity(joint_handle, velocity / radius);
}

void WheelDriver::setDriverEffort(double effort)
{
  sim->setEffort(joint_handle, effort / radius);
}

void WheelDriver::initializeChild()
{
  radius = get<0>(sim->getObjectSize(link_handle)) / 2.0;
}
