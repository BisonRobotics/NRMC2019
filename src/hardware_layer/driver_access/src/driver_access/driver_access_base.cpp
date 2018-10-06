#include <driver_access/driver_access_base.h>
#include <driver_access/clamp.h>
#include <cmath>
#include <algorithm>

using namespace driver_access;

using std::abs;
using std::to_string;

DriverAccessBase::DriverAccessBase(const Limits &limits, uint8_t id, std::string name) :
    limits(limits), id(id), name(name) {};

DriverAccessBase::DriverAccessBase(const Limits &limits, uint8_t id) :
    DriverAccessBase(limits, id, "driver" + to_string(id)) {};

DriverAccessBase::DriverAccessBase(const Limits &limits) :
    DriverAccessBase(limits, 0) {};

void DriverAccessBase::setPosition(double position)
{
  setDriverPosition(clamp(position, limits.min_position, limits.max_position));
}

void DriverAccessBase::setVelocity(double velocity)
{
  setDriverVelocity(absClamp(velocity, limits.min_velocity, limits.max_velocity));
}

void DriverAccessBase::setEffort(double effort)
{
  setDriverEffort(absClamp(effort, limits.min_effort, limits.max_effort));
}


