#ifndef DRIVER_ACCESS_DRIVER_ACCESS_ERROR_H
#define DRIVER_ACCESS_DRIVER_ACCESS_ERROR_H

#include <stdexcept>

namespace driver_access
{

struct driver_access_error : public std::logic_error
{
  driver_access_error(std::string const &message) : std::logic_error(message) {};
};

}


#endif //DRIVER_ACCESS_DRIVER_ACCESS_ERROR_H
