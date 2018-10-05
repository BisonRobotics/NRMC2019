#include <vrep_wheels/vrep_wheels.h>


using namespace vrep_interface;


VREPWheels::VREPWheels() {}


simInt VREPWheels::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    this->handles[i]= -1;
  }
  this->setDistance(3.4290e-01 * 2, 3.429e-01 * 2);
}


simInt VREPWheels::updateWheelIDs()
{
  for (int i = 0; i < 4; i++)
  {
    handles[i] = simGetObjectHandle(this->joint_names[i]);
    if (handles[i] == -1)
    {
      std::string message = "Unable to find joint \"" + std::string(this->joint_names[i]) + "\", "
          "make sure the joints are named the same in vrep "
          "and in vrep_state.cpp";
      simAddStatusbarMessage(("[method updateWheelIDs] " + message).c_str());
      return (simInt)false;
    }
    simAddStatusbarMessage(("[method updateWheelIDs] Found an id of "
                            + std::to_string(handles[i])
                            + " for \"" + std::string(this->joint_names[i]) + "\"").c_str());
  }
  return (simInt)true;
}


void VREPWheels::setPosition(int index, double position)
{

}


void VREPWheels::setVelocity(int index, double velocity)
{
  //TODO properly scale velocity
  if(simSetJointTargetVelocity(handles[index], (simFloat)(velocity / 0.1524)) == -1)
  {
    simAddStatusbarMessage(("[Robot::spinOnce] unable to set target velocity for " +
                            std::string(joint_names[index])).c_str());
  }
}


void VREPWheels::setEffort(int index, double effort)
{

}


double VREPWheels::getPosition(int index)
{
  simFloat position;
  simGetJointPosition(handles[index], &position);
  return (double)position;
}


double VREPWheels::getVelocity(int index)
{
  //TODO properly scale velocity
  simFloat velocity;
  simGetObjectFloatParameter(handles[index], 2012, &velocity);
  return (double)(velocity * 0.1524f);
}

double VREPWheels::getEffort(int index)
{
  simFloat effort;
  simGetJointForce(handles[index], &effort);
  return (double)effort;
}


