#include <vrep_interface/vrep_interface.h>
#include <vrep_library/v_repLib.h>

using namespace vrep_interface;

simInt VREPInterface::vSimAddStatusBarMessage(const simChar *message)
{
  return simAddStatusbarMessage(message);
}

simInt VREPInterface::vSimGetObjectHandle(const simChar *object_name)
{
  return simGetObjectHandle(object_name);
}

simInt VREPInterface::vSimGetJointPosition(simInt object_handle, simFloat *position)
{
  return simGetJointPosition(object_handle, position);
}

simInt VREPInterface::vSimSetJointPosition(simInt object_handle, simFloat position)
{
  return simSetJointPosition(object_handle, position);
}

simInt VREPInterface::vSimGetObjectFloatParameter(simInt object_handle,
    simInt parameter_id, simFloat *parameter)
{
  return simGetObjectFloatParameter(object_handle, parameter_id, parameter);
}

simInt VREPInterface::vSimGetJointForce(simInt object_handle, simFloat *force)
{
  return simGetJointForce(object_handle, force);
}

simInt VREPInterface::vSimSetJointTargetVelocity(simInt object_handle, simFloat velocity)
{
  return simSetJointTargetVelocity(object_handle, velocity);
}

simInt VREPInterface::vSimLoadScene(const simChar *filename)
{
  return simLoadScene(filename);
}

simInt VREPInterface::vSimSetIntegerParameter(simInt parameter, simInt int_state)
{
  return simSetIntegerParameter(parameter, int_state);
}

simInt VREPInterface::vSimGetIntegerParameter(simInt parameter, simInt *int_state)
{
  return simGetIntegerParameter(parameter, int_state);
}

simFloat VREPInterface::vSimGetSimulationTime()
{
  return simGetSimulationTime();
}

