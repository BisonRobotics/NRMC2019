#ifndef DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H
#define DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H

#include <stdint.h>
#include <string>

namespace driver_access
{

class DriverAccessInterface
{
  public:
    virtual void setVelocity(double velocity) = 0; // m/s
    virtual void setTorque(double torque) = 0; // nm
    virtual void setPosition(double position) = 0; // rad

    virtual double getVelocity(void) = 0;
    virtual double getTorque(void) = 0;
    virtual double getPosition(void) = 0;

  protected:
    virtual void setDriverVelocity(double velocity) = 0; // m/s
    virtual void setDriverTorque(double torque) = 0; // nm
    virtual void setDriverPosition(double position) = 0; // rad
};

}

#endif //DRIVER_ACCESS_DRIVER_ACCESS_INTERFACE_H
