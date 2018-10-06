#ifndef DRIVER_ACCESS_MODE_H
#define DRIVER_ACCESS_MODE_H

#include <cstdint>

namespace driver_access
{
  namespace Mode
  {
    enum : uint8_t {velocity, torque, position};
  }
}

#endif //DRIVER_ACCESS_MODE_H
