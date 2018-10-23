#ifndef VREP_INTERFACE_VREP_ROBOT_H
#define VREP_INTERFACE_VREP_ROBOT_H

#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <tf/tf.h>

#include <vrep_library/v_repLib.h>
#include <vrep_drivers/vrep_wheel_driver.h>

namespace vrep_interface
{

const double mining_zone_centers[2] = {0.945, -0.945};
const double M_2PI_6 = (2.0 * M_PI) / 6.0;
const double mining_zone_rotations[6] =
{
  0.0 * M_2PI_6,
  1.0 * M_2PI_6,
  2.0 * M_2PI_6,
  3.0 * M_2PI_6,
  4.0 * M_2PI_6,
  5.0 * M_2PI_6
};

class VREPRobot
{
  public:

    VREPRobot(SimInterface *sim_interface);

    void initialize(std::string model_file);
    void spawnRobot();
    void spawnRobot(double x, double y, double rotation);
    void checkState();
    void loadModel();
    void getPosition(tf::Transform *position);
    void updateWheelHandles();
    void spinOnce();
    void shutdown();

  private:
    SimInterface *sim;
    int handle;
    int base_link_handle;
    std::string model_file;
    VREPWheelDriver fl, bl, fr, br;

    void loadModelHelper();
};

}


#endif // VREP_INTERFACE_VREP_ROBOT_H
