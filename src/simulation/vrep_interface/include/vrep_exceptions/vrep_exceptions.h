#ifndef PROJECT_VREP_EXCEPTIONS_H
#define PROJECT_VREP_EXCEPTIONS_H

#include <iostream>
#include <exception>

namespace vrep_interface
{

class vrep_error : public std::runtime_error
{
public:
  vrep_error() : std::runtime_error("Error occurred in VREP") {};
  vrep_error(std::string msg) : std::runtime_error(msg) {};
};

class get_object_attribute_error : public vrep_error
{
public:
  get_object_attribute_error() : vrep_error("Unable to find an object attribute in VREP") {};
  get_object_attribute_error(std::string msg) : vrep_error(msg) {};
};

class set_object_attribute_error : public vrep_error
{
public:
  set_object_attribute_error() : vrep_error("Unable to set an object attribute in VREP") {};
  set_object_attribute_error(std::string msg) : vrep_error(msg) {};
};
}

#endif //PROJECT_VREP_EXCEPTIONS_H
