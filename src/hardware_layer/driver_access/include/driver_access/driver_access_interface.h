#ifndef DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H
#define DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H

#include <driver_access/mode.h>

namespace driver_access
{

class DriverAccessInterface
{
  public:
    virtual void setPosition(double position) = 0;
    virtual void setVelocity(double velocity) = 0;
    virtual void setEffort(double effort) = 0;

    virtual void setPoint(double value) = 0;
    virtual void setMode(Mode mode) = 0;
    virtual Mode getMode() = 0;

    virtual double getPosition() = 0;
    virtual double getVelocity() = 0;
    virtual double getEffort() = 0;

  protected:
    virtual void setDriverPosition(double position) = 0;
    virtual void setDriverVelocity(double velocity) = 0;
    virtual void setDriverEffort(double force) = 0;
};

}

#endif //DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H
