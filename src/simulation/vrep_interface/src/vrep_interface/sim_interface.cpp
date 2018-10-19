#include <vrep_interface/sim_interface.h>
#include <vrep_library/v_repConst.h>

using namespace vrep_interface;

void SimInterface::setPosition(int object_handle, double position)
{
  this->vSimSetJointPosition((simInt) object_handle, (simFloat) position);
}

void SimInterface::setEffort(int object_handle, double effort)
{
  throw std::runtime_error("[SimInterface]: setEffort not implemented");
}

void SimInterface::setVelocity(int object_handle, double velocity)
{
  this->vSimSetJointTargetVelocity((simInt) object_handle, (simFloat) velocity);
}

double SimInterface::getPosition(int object_handle)
{
  simFloat position;
  this->vSimGetJointPosition((simInt) object_handle, &position);
  return position;
}

double SimInterface::getEffort(int object_handle)
{
  simFloat effort;
  this->vSimGetJointForce((simInt) object_handle, &effort);
  return effort;
}

double SimInterface::getVelocity(int object_handle)
{
  simFloat velocity;
  this->vSimGetObjectFloatParameter(object_handle, 2012, &velocity);
  return velocity;
}

void SimInterface::loadScene(const std::string &filename)
{
  this->vSimLoadScene(filename.c_str());
}

void SimInterface::turnOffErrorReporting()
{
  this->vSimGetIntegerParameter(sim_intparam_error_report_mode, &last_error_mode);
  this->vSimSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
}

void SimInterface::turnOnErrorReporting()
{
  this->vSimSetIntegerParameter(sim_intparam_error_report_mode, last_error_mode);
}

rosgraph_msgs::Clock SimInterface::getSimulationTime()
{
  ros::Time sim_time(this->vSimGetSimulationTime());
  rosgraph_msgs::Clock sim_time_msg;
  sim_time_msg.clock.nsec = sim_time.nsec;
  sim_time_msg.clock.sec = sim_time.sec;
  return sim_time_msg;
}
