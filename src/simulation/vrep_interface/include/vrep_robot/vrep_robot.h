#ifndef PROJECT_VREPROBOT_H
#define PROJECT_VREPROBOT_H


#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <tf/tf.h>

#include <vrep_library/v_repLib.h>
#include <vrep_drivers/vrep_wheel_driver.h>

namespace vrep_interface
{


const simFloat mining_zone_centers[2] = {0.945f, -0.945f};
const simFloat M_2PI_6 = (simFloat)((2 * M_PI) / 6);
const simFloat mining_zone_rotations[6] =
{
  0.0f * M_2PI_6,
  1.0f * M_2PI_6,
  2.0f * M_2PI_6,
  3.0f * M_2PI_6,
  4.0f * M_2PI_6,
  5.0f * M_2PI_6
};


class VREPRobot
{
  public:

    VREPRobot();

    void initialize(std::string model_file);
    void spawnRobot();
    void spawnRobot(simFloat x, simFloat y, simFloat rotation);
    void checkState();
    void loadModel();
    void getPosition(tf::Transform *position);
    void move(simFloat x, simFloat y);
    void rotate(simFloat rotation);
    void updateWheelHandles();
    void spinOnce();
    void shutdown();

  private:
    simInt handle;
    simInt base_link_handle;
    std::string model_file;
    VREPWheelDriver fl;
    VREPWheelDriver bl;
    VREPWheelDriver fr;
    VREPWheelDriver br;

    void loadModelHelper();

};




}


#endif //PROJECT_ROBOT_H
