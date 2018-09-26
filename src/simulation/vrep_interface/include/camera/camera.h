#ifndef PROJECT_CAMERA_H
#define PROJECT_CAMERA_H

#include <string>
#include <vrep_lib/v_repLib.h>

namespace vrep_interface
{

class Camera
{
public:
  Camera(std::string name);
  void get_position();
  void set_position(simFloat position[3], simFloat orientation[3]);
  simFloat position[3];
  simFloat orientation[3];

private:
  std::string name;
  simInt handle;
};
}

#endif //PROJECT_CAMERA_H
