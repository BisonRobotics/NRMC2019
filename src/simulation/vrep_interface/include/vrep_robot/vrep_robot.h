#ifndef PROJECT_VREPROBOT_H
#define PROJECT_VREPROBOT_H


#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <pluginlib/class_loader.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

#include <vrep_lib/v_repLib.h>
#include <vrep_wheels/vrep_wheels.h>
#include <wheel_control/wheels/wheels.h>
#include <wheel_control/velocity_interface/velocity_interface.h>


namespace vrep_interface
{


const simFloat mining_zone_centers[2][2] =
{
  {0.75f,  0.945f},
  {0.75f, -0.945f}
};
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
  VREPRobot(std::string model_file);

  void spinOnce();
  void setVelocity(double linear, double angular);
  void initialize(wheel_control::Wheels *wheels, wheel_control::VelocityInterface *controller);
  void getPosition(tf::Transform *position);

  simInt setModelFile(std::string model_file);
  simInt spawnRobot();
  simInt spawnRobot(simFloat *position, simFloat rotation);
  simInt checkState();
  simInt loadModel();
  simInt move(simFloat position[2]);
  simInt rotate(simFloat rotation);
  simInt initializeWheels();
  //simInt get_joint_states(sensor_msgs::JointState *joint_states);

private:
  simInt handle;
  simInt base_link_handle;
  std::string model_file;
  VREPWheels wheels;
  uint32_t scan_seq;

  simInt loadModelHelper();

};




}


#endif //PROJECT_ROBOT_H
