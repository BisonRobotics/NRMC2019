#include <driver_access/driver_access.h>
#include <driver_access/clamp.h>
#include <cmath>
#include <algorithm>
#include <driver_access/mode.h>

using namespace driver_access;

using std::abs;

DriverAccess::DriverAccess(const Limits &limits, ID id, Mode mode) :
    limits(limits), id(id), mode(mode) {};

void DriverAccess::setPosition(double position)
{
  if (mode == Mode::none || mode == Mode::position)
  {
    setDriverPosition(clamp(position, limits.min_position, limits.max_position));
  }
  else
  {
    throw mode_error("Expected a mode of none or position");
  }
}

void DriverAccess::setVelocity(double velocity)
{
  if (mode == Mode::none || mode == Mode::velocity)
  {
    setDriverVelocity(absClamp(velocity, limits.min_velocity, limits.max_velocity));
  }
  else
  {
    throw mode_error("Expected a mode of none or velocity");
  }
}

void DriverAccess::setEffort(double effort)
{
  if (mode == Mode::none || mode == Mode::effort)
  {
    setDriverEffort(absClamp(effort, limits.min_effort, limits.max_effort));
  }
  else
  {
    throw mode_error("Expected a mode of none or effort");
  }
}

void DriverAccess::setPoint(double value)
{
  if (mode == Mode::position)
  {
    setPosition(value);
  }
  else if (mode == Mode::velocity)
  {
    setVelocity(value);
  }
  else if (mode == Mode::effort)
  {
    setEffort(value);
  }
  else
  {
    throw mode_error("No mode currently selected");
  }
}

void DriverAccess::setMode(Mode mode)
{
  this->mode = mode;

}

Mode DriverAccess::getMode()
{
  return mode;
}



