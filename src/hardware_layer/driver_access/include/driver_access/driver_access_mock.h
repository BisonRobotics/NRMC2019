#ifndef DRIVER_ACCESS_DRIVER_ACCESS_MOCK_H
#define DRIVER_ACCESS_DRIVER_ACCESS_MOCK_H

#include <driver_access/driver_access.h>
#include <driver_access/params.h>
#include <gmock/gmock.h>

namespace driver_access
{

class DriverAccessMock : public DriverAccess
{
  public:
    DriverAccessMock(const Limits &limits) : DriverAccess(limits) {};
    DriverAccessMock(const Limits &limits, Mode mode) : DriverAccess(limits, ID::none, mode) {};

    MOCK_METHOD1(setDriverPosition, void(double position));
    MOCK_METHOD1(setDriverVelocity, void(double velocity));
    MOCK_METHOD1(setDriverEffort, void(double effort));

    MOCK_METHOD0(getPosition, double(void));
    MOCK_METHOD0(getVelocity, double(void));
    MOCK_METHOD0(getEffort, double(void));
};

}

#endif //DRIVER_ACCESS_DRIVER_ACCESS_MOCK_H
