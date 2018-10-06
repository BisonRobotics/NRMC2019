#ifndef DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H
#define DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H

#include <stdint.h>
#include <string>

namespace driver_access
{

class DriverAccessInterface
{
  public:
    virtual void setPosition(double position) = 0;
    virtual void setVelocity(double velocity) = 0;
    virtual void setEffort(double effort) = 0;

    virtual double getPosition(void) = 0;
    virtual double getVelocity(void) = 0;
    virtual double getEffort(void) = 0;

  protected:
    virtual void setDriverPosition(double position) = 0;
    virtual void setDriverVelocity(double velocity) = 0;
    virtual void setDriverEffort(double force) = 0;
};

}

#endif //DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H
