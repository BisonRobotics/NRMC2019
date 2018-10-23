#ifndef VREP_PLUGIN_WHEEL_DRIVER_H
#define VREP_PLUGIN_WHEEL_DRIVER_H

#include <vrep_drivers/driver_base.h>

namespace vrep_plugin
{

class WheelDriver : public DriverBase
{
  public:
    WheelDriver(Interface *sim_interface, driver_access::ID id);

    void updateState() override;
    void initializeChild() override;

  protected:
    void setDriverPosition(double position) override;
    void setDriverVelocity(double velocity) override;
    void setDriverEffort(double torque) override;

  private:
    double radius;
};
}

#endif //VREP_PLUGIN_WHEEL_DRIVER_H
