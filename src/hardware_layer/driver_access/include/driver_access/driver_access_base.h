#ifndef DRIVER_ACCESS_DRIVER_ACCESS_BASE_H
#define DRIVER_ACCESS_DRIVER_ACCESS_BASE_H

#include <driver_access/driver_access_interface.h>
#include <driver_access/limits.h>

namespace driver_access
{

class DriverAccessBase : public DriverAccessInterface
{
  public:
    const Limits limits;
    const uint8_t id;
    const std::string name;

    DriverAccessBase(const Limits &limits);
    DriverAccessBase(const Limits &limits, uint8_t id);
    DriverAccessBase(const Limits &limits, uint8_t id, std::string name);

    void setPosition(double position) override; // rad
    void setVelocity(double velocity) override; // m/s
    void setEffort(double effort) override;
};

}


#endif //DRIVER_ACCESS_DRIVER_ACCESS_BASE_H
