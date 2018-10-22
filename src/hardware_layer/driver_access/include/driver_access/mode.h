#ifndef DRIVER_ACCESS_MODE_H
#define DRIVER_ACCESS_MODE_H

#include <driver_access/driver_access_error.h>

namespace driver_access
{

struct mode_error : public driver_access_error
{
  mode_error(std::string const &message) : driver_access_error(message) {};
};

enum class Mode : uint8_t {none, position, velocity, effort};

}

#endif //DRIVER_ACCESS_MODE_H
