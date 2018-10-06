#ifndef VREP_INTERFACE_VREP_WHEEL_DRIVER_H
#define VREP_INTERFACE_VREP_WHEEL_DRIVER_H

#include <vrep_drivers/vrep_driver.h>

namespace vrep_interface
{

class VREPWheelDriver : public VREPDriver
{
  public:
    VREPWheelDriver(uint8_t id, const std::string &joint_name);

    void updateState() override;

  protected:
    void setDriverPosition(double position) override;
    void setDriverVelocity(double velocity) override;
    void setDriverEffort(double torque) override;
};
}

#endif //VREP_INTERFACE_VREP_WHEEL_DRIVER_H
