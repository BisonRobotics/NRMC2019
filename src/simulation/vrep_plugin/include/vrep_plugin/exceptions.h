#ifndef VREP_PLUGIN_EXCEPTIONS_H
#define VREP_PLUGIN_EXCEPTIONS_H

#include <iostream>
#include <exception>

namespace vrep_plugin
{

class vrep_error : public std::runtime_error
{
  public:
    vrep_error() : std::runtime_error("Error occurred in VREP") {};
    vrep_error(std::string msg) : std::runtime_error(msg) {};
};

}

#endif //VREP_PLUGIN_EXCEPTIONS_H
