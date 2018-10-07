#ifndef DRIVER_ACCESS_DRIVER_ACCESS_BASE_H
#define DRIVER_ACCESS_DRIVER_ACCESS_BASE_H

#include <driver_access/driver_access_interface.h>
#include <driver_access/params.h>
#include <driver_access/limits.h>
#include <boost/shared_ptr.hpp>
#include <cstdint>

namespace driver_access
{

class DriverAccess : public DriverAccessInterface
{
  public:
    const Limits limits;
    const ID id;

    explicit DriverAccess(const Limits &limits, ID id = ID::none, Mode mode = Mode::none);

    // Use a specific control mode
    void setPosition(double position) override; // rad
    void setVelocity(double velocity) override; // m/s
    void setEffort(double effort) override;

    // Use previously specified control mode
    void setPoint(double value) override;
    void setMode(Mode mode) override;
    Mode getMode() override;

  private:
    Mode mode;
};

}


#endif //DRIVER_ACCESS_DRIVER_ACCESS_BASE_H
