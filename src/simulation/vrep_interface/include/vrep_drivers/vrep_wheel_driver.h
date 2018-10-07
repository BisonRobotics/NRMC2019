#ifndef VREP_INTERFACE_VREP_WHEEL_DRIVER_H
#define VREP_INTERFACE_VREP_WHEEL_DRIVER_H

#include <vrep_drivers/vrep_driver_base.h>

namespace vrep_interface
{

class VREPWheelDriver : public VREPDriverBase
{
  public:
    VREPWheelDriver(driver_access::ID id);

    void updateState() override;

  protected:
    void setDriverPosition(double position) override;
    void setDriverVelocity(double velocity) override;
    void setDriverEffort(double torque) override;
};
}

#endif //VREP_INTERFACE_VREP_WHEEL_DRIVER_H
