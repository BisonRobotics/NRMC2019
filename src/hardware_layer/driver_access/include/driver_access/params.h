#ifndef DRIVER_ACCESS_PARAMS_H
#define DRIVER_ACCESS_PARAMS_H

#include <cstdint>
#include <string>
#include <vector>

namespace driver_access
{
enum class ID : uint8_t
{
    none = 0,
    front_left_wheel = 1,
    front_right_wheel = 2,
    back_right_wheel = 3,
    back_left_wheel = 4,
    central_drive = 5,
    linear_motor = 6
};

static inline std::string name(uint8_t id)
{
  static const std::string names[6] = {"front_left_wheel", "front_right_wheel",
                                       "back_right_wheel", "back_left_wheel",
                                       "central_drive", "linear_actuator"};
  if (id < 1 || id > 6)
  {
    return "";
  }
  return names[id-1];
}

static inline std::string name(ID id)
{
  return name(static_cast<uint8_t>(id));
}

}

#endif //DRIVER_ACCESS_PARAMS_H
