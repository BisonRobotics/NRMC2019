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

simInt VREPInterface::vSimLoadScene(std::string filename)
{
  return simLoadScene(filename.c_str());
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

simInt VREPInterface::vSimIsHandleValid(simInt object_handle, simInt object_type)
{
  return simIsHandleValid(object_handle, object_type);
}

simInt VREPInterface::vSimLoadModel(std::string filename)
{
  std::cout << "vSimLoadModel" << std::endl;
  return simLoadModel(filename.c_str());
}

simInt* VREPInterface::vSimGetObjectsInTree(simInt handle, simInt type, simInt options, simInt *count)
{
  return simGetObjectsInTree(handle, type, options, count);
}

simChar* VREPInterface::vSimGetObjectName(simInt handle)
{
  return simGetObjectName(handle);
}

simInt VREPInterface::vSimReleaseBuffer(simChar *buffer)
{
  return simReleaseBuffer(buffer);
}

simInt VREPInterface::vSimRemoveModel(simInt handle)
{
  return simRemoveModel(handle);
}

simInt VREPInterface::vSimSetObjectPosition(simInt handle,
    simInt relative_to_handle, const simFloat *position)
{
  return simSetObjectPosition(handle, relative_to_handle, position);
}

simInt VREPInterface::vSimSetObjectOrientation(simInt handle,
    simInt relative_to_handle, const simFloat *orientation)
{
  return simSetObjectOrientation(handle, relative_to_handle, orientation);
}

simInt VREPInterface::vSimGetObjectPosition(simInt handle,
    simInt relative_to_handle, simFloat *position)
{
  return simGetObjectPosition(handle, relative_to_handle, position);
}

simInt VREPInterface::vSimGetObjectOrientation(simInt handle,
    simInt relative_to_handle, simFloat *orientation)
{
  return simGetObjectOrientation(handle, relative_to_handle, orientation);
}

