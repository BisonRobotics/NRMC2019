#ifndef DRIVER_ACCESS_DRIVER_ACCESS_MOCK_H
#define DRIVER_ACCESS_DRIVER_ACCESS_MOCK_H

#include <driver_access/driver_access_base.h>
#include <gmock/gmock.h>

namespace driver_access
{

class DriverAccessMock : public DriverAccessBase
{
  public:
    DriverAccessMock(const Limits &limits) : DriverAccessBase(limits) {};

    MOCK_METHOD1(setDriverVelocity, void(double velocity));
    MOCK_METHOD1(setDriverTorque, void(double velocity));
    MOCK_METHOD1(setDriverPosition, void(double velocity));

    MOCK_METHOD0(getVelocity, double(void));
    MOCK_METHOD0(getTorque, double(void));
    MOCK_METHOD0(getPosition, double(void));
};

}

#endif //DRIVER_ACCESS_DRIVER_ACCESS_MOCK_H
