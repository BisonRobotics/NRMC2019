#include <camera/camera.h>
#include <vrep_exceptions/vrep_exceptions.h>


using namespace vrep_interface;


Camera::Camera(std::string name)
{
  this->name = name;
  handle = simGetObjectHandle(name.c_str());
  if (handle == -1)
  {
    throw get_object_attribute_error("Unable to find " + name + " in VREP");
  }
  get_position();
}


void Camera::get_position()
{
  simInt status = -1;

  status = simGetObjectPosition(handle, -1, position);
  if (status == -1)
  {
    throw get_object_attribute_error("Unable to get object position for " + name + "(" + std::to_string(handle)
                                     + ") in VREP");
  }

  status = simGetObjectOrientation(handle, -1, orientation);
  if (status == -1)
  {
    throw get_object_attribute_error("Unable to get object orientation for " + name + "(" + std::to_string(handle)
                                     + ") in VREP");
  }
}


void Camera::set_position(simFloat position[3], simFloat orientation[3])
{
  simInt status = -1;

  status = simSetObjectPosition(handle, -1, position);
  if (status == -1)
  {
    throw set_object_attribute_error("Unable to set object position for " + name + "(" + std::to_string(handle)
                                     + ") in VREP");
  }

  status = simSetObjectOrientation(handle, -1, orientation);
  if (status == -1)
  {
    throw set_object_attribute_error("Unable to set object orientation for " + name + "(" + std::to_string(handle)
                                     + ") in VREP");
  }
}
