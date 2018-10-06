#ifndef PROJECT_VREP_WHEELS_H
#define PROJECT_VREP_WHEELS_H


#include <vrep_lib/v_repLib.h>

namespace vrep_interface
{

class VREPWheels
{
public:
  const char *joint_names[4] = {"wheel_front_left_joint", "wheel_front_right_joint",
                                "wheel_back_left_joint", "wheel_back_right_joint"};
  const char *wheel_names[4] = {"wheel_front_left", "wheel_front_right",
                                "wheel_back_left", "wheel_back_right"};
  int handles[4];

  VREPWheels();
  simInt initialize();
  simInt updateWheelIDs();

  void setPosition(int index, double position);
  void setVelocity(int index, double velocity);
  void setEffort(int index, double effort);

  double getPosition(int index);
  double getVelocity(int index);
  double getEffort(int index);
};

}
#endif //PROJECT_VREP_WHEELS_H
