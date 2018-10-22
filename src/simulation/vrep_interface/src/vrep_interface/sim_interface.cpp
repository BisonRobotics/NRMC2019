#include <vrep_interface/sim_interface.h>
#include <vrep_library/v_repConst.h>
#include <vrep_exceptions/vrep_exceptions.h>
#include <vrep_library/v_repLib.h>

using namespace vrep_interface;

SimInterface::SimInterface() : last_error_mode(0) {}

void SimInterface::setJointPosition(int object_handle, double position)
{
  if (vSimSetJointPosition((simInt) object_handle, (simFloat) position) == -1)
  {
    throw vrep_error("[SimInterface::setPosition]: Unable to set position for object: "
                     + std::to_string(object_handle));
  }
}

void SimInterface::setEffort(int object_handle, double effort)
{
  throw vrep_error("[SimInterface::setEffort]: Not implemented");
}

void SimInterface::setVelocity(int object_handle, double velocity)
{
  if (vSimSetJointTargetVelocity((simInt) object_handle, (simFloat) velocity) == -1)
  {
    throw vrep_error("[SimInterface::setVelocity]: Unable to set velocity for object: "
                     + std::to_string(object_handle));
  }
}

double SimInterface::getPosition(int object_handle)
{
  simFloat position;
  if (vSimGetJointPosition((simInt) object_handle, &position) == -1)
  {
    throw vrep_error("[SimInterface::getPosition]: Unable to get position for object: "
                     + std::to_string(object_handle));
  }
  return position;
}

double SimInterface::getEffort(int object_handle)
{
  simFloat effort;
  if (vSimGetJointForce((simInt) object_handle, &effort) == -1)
  {
    throw vrep_error("[SimInterface::getEffort]: Unable to get effort for object: "
                     + std::to_string(object_handle));
  }
  return effort;
}

double SimInterface::getVelocity(int object_handle)
{
  simFloat velocity;
  if (vSimGetObjectFloatParameter(object_handle, 2012, &velocity) == -1)
  {
    throw vrep_error("[SimInterface::getVelocity]: Unable to get velocity for object: "
                     + std::to_string(object_handle));
  }
  return velocity;
}

int SimInterface::getObjectHandle(const std::string &object_name)
{
  int object_handle = vSimGetObjectHandle(object_name.c_str());
  if (object_handle == -1)
  {
    throw vrep_error("[SimInterface::getObjectHandle]: "
                     "Unable to find object handle for: " + object_name);
  }
  return object_handle;
}

int SimInterface::loadScene(const std::string &filename)
{
  int handle = vSimLoadScene(filename.c_str());
  if(handle == -1)
  {
    throw vrep_error("[SimInterface::loadScene]: Unable to load scene: " + filename);
  }
  return handle;
}

void SimInterface::disableErrorReporting()
{
  if (vSimGetIntegerParameter(sim_intparam_error_report_mode, &last_error_mode) == -1)
  {
    throw vrep_error("[SimInterface::disableErrorReporting]: "
                     "Unable to get error report mode");
  }
  if (vSimSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore) == -1)
  {
    throw vrep_error("[SimInterface::disableErrorReporting]: "
                     "Unable to set error report mode");
  }
}

void SimInterface::resumeErrorReporting()
{
  if (vSimSetIntegerParameter(sim_intparam_error_report_mode, last_error_mode) == -1)
  {
    throw vrep_error("[SimInterface::resumeErrorReporting]: "
                     "Unable to set error report mode");
  }
}

rosgraph_msgs::Clock SimInterface::getSimulationTime()
{
  simFloat sim_time_f = vSimGetSimulationTime();
  if (sim_time_f < 0.0f)
  {
    throw vrep_error("[SimInterface::getSimulationTime]: "
                     "Unable to get simulation time");
  }
  ros::Time sim_time(sim_time_f);
  rosgraph_msgs::Clock sim_time_msg;
  sim_time_msg.clock.nsec = sim_time.nsec;
  sim_time_msg.clock.sec = sim_time.sec;
  return sim_time_msg;
}

void SimInterface::info(const std::string &message)
{
  if (vSimAddStatusBarMessage(("[INFO]: " + message).c_str()) == -1)
  {
    throw vrep_error("[SimInterface::info]: "
                     "Unable add a status bar message");
  }
}

void SimInterface::warn(const std::string &message)
{
  if (vSimAddStatusBarMessage(("[WARN]: " + message).c_str()) == -1)
  {
    throw vrep_error("[SimInterface::warn]: "
                     "Unable add a status bar message");
  }
}

void SimInterface::error(const std::string &message)
{
  if (vSimAddStatusBarMessage(("[ERROR]: " + message).c_str()) == -1)
  {
    throw vrep_error("[SimInterface::error]: "
                     "Unable add a status bar message");
  }
}

bool SimInterface::isHandleValid(int object_handle, int object_type)
{
  return (bool)vSimIsHandleValid(object_handle, object_type);
}

int SimInterface::loadModel(const std::string &filename)
{
  int handle = vSimLoadModel(filename.c_str());
  if (handle == -1)
  {
    throw vrep_error("[SimInterface::loadModel]: Unable to load model: " + filename);
  }
  return handle;
}

int SimInterface::findObjectInTree(int base_handle, const std::string &object_name, int object_type)
{
  int handle = -1;
  simInt *tree_handles;
  simInt handle_count = -1;
  tree_handles = vSimGetObjectsInTree(base_handle, object_type, 0x00, &handle_count);
  for (int i = 0; i < handle_count; i++)
  {
    simChar *name_c = vSimGetObjectName(tree_handles[i]);
    if (name_c != NULL)
    {
      std::string name = std::string(name_c);
      if (name == object_name)
      {
        handle = tree_handles[i];
      }
    }
  }
  if (handle == -1)
  {
    throw vrep_error("[SimInterface::findObjectInTree]: Unable to find object: " +
                     object_name);
  }
  if (vSimReleaseBuffer((simChar*)tree_handles) == -1)
  {
    throw vrep_error("[SimInterface::findObjectInTree]: Unable to release buffer");
  }

  return handle;
}

void SimInterface::removeModel(int object_handle)
{
  if (vSimRemoveModel(object_handle) == -1)
  {
    throw vrep_error("[SimInterface::removeModel]: Unable to remove model: " +
                     std::to_string(object_handle));
  }
}

void SimInterface::setObjectPosition(int handle, int relative_to_handle,
    double x, double y, double z)
{
  simFloat position[]{(simFloat)x, (simFloat)y, (simFloat)z};
  if (vSimSetObjectPosition((simInt)handle, (simInt)relative_to_handle, position) == -1)
  {
    throw vrep_error("[SimInterface::setObjectPosition]: "
                     "Unable to set object position: " + std::to_string(handle));
  }
}

void SimInterface::setObjectOrientation(int handle, int relative_to_handle, double alpha, double beta, double gamma)
{
  simFloat orientation[]{(simFloat)alpha, (simFloat)beta, (simFloat)gamma};
  if (vSimSetObjectOrientation((simInt)handle, (simInt)relative_to_handle, orientation) == -1)
  {
    throw vrep_error("[SimInterface::setObjectOrientation]: "
                     "Unable to set object orientation: " + std::to_string(handle));
  }
}

tuple3d SimInterface::getObjectPosition(int handle, int relative_to_handle)
{
  simFloat position[3];
  if (vSimGetObjectPosition((simInt)handle, (simInt)relative_to_handle, position) == -1)
  {
    throw vrep_error("[SimInterface::getObjectPosition]: "
                     "Unable to get object position");
  }
  return std::make_tuple((double)position[0], (double)position[1], (double)position[2]);
}

tuple3d SimInterface::getObjectOrientation(int handle, int relative_to_handle)
{
  simFloat orientation[3];
  if (vSimGetObjectPosition((simInt)handle, (simInt)relative_to_handle, orientation) == -1)
  {
    throw vrep_error("[SimInterface::getObjectOrientation]: "
                     "Unable to get object orientation");
  }
  return std::make_tuple((double)orientation[0], (double)orientation[1], (double)orientation[2]);
}

double SimInterface::getFloatParameter(int handle, int parameter_id)
{
  simFloat parameter;
  if (vSimGetObjectFloatParameter((simInt)handle, (simInt)parameter_id, &parameter) == -1)
  {
    info("[SimInterface::getFloatParameter]: "
         "Unable to get parameter: " + std::to_string(parameter_id));
    throw vrep_error("[SimInterface::getFloatParameter]: "
                     "Unable to get parameter: " + std::to_string(parameter_id));
  }
  return (double)parameter;
}

void SimInterface::setParameter(int handle, int parameter_id, double value)
{
  if (vSimSetObjectFloatParameter((simInt) handle, (simInt) parameter_id, (simFloat) value) == -1)
  {
    throw vrep_error("[SimInterface::setParameter]: "
                     "Unable to set parameter: " + std::to_string(parameter_id) +
                     " to: " + std::to_string(value));
  }
}

simInt SimInterface::vSimAddStatusBarMessage(const simChar *message)
{
  return simAddStatusbarMessage(message);
}

simInt SimInterface::vSimGetObjectHandle(const simChar *object_name)
{
  return simGetObjectHandle(object_name);
}

simInt SimInterface::vSimGetJointPosition(simInt object_handle, simFloat *position)
{
  return simGetJointPosition(object_handle, position);
}

simInt SimInterface::vSimSetJointPosition(simInt object_handle, simFloat position)
{
  return simSetJointPosition(object_handle, position);
}

simInt SimInterface::vSimGetObjectFloatParameter(simInt object_handle, simInt parameter_id, simFloat *parameter)
{
  return simGetObjectFloatParameter(object_handle, parameter_id, parameter);
}

simInt SimInterface::vSimSetObjectFloatParameter(simInt handle, simInt parameter_id, simFloat parameter)
{
  return simSetObjectFloatParameter(handle, parameter_id, parameter);
}

simInt SimInterface::vSimGetJointForce(simInt object_handle, simFloat *force)
{
  return simGetJointForce(object_handle, force);
}

simInt SimInterface::vSimSetJointTargetVelocity(simInt object_handle, simFloat velocity)
{
  return simSetJointTargetVelocity(object_handle, velocity);
}

simInt SimInterface::vSimLoadScene(const simChar *filename)
{
  return simLoadScene(filename);
}

simInt SimInterface::vSimSetIntegerParameter(simInt parameter, simInt int_state)
{
  return simSetIntegerParameter(parameter, int_state);
}

simInt SimInterface::vSimGetIntegerParameter(simInt parameter, simInt *int_state)
{
  return simGetIntegerParameter(parameter, int_state);
}

simFloat SimInterface::vSimGetSimulationTime()
{
  return simGetSimulationTime();
}

simInt SimInterface::vSimIsHandleValid(simInt object_handle, simInt object_type)
{
  return simIsHandleValid(object_handle, object_type);
}

simInt SimInterface::vSimLoadModel(const simChar *filename)
{
  return simLoadModel(filename);
}

simInt* SimInterface::vSimGetObjectsInTree(simInt handle, simInt type, simInt options, simInt *count)
{
  return simGetObjectsInTree(handle, type, options, count);
}

simChar* SimInterface::vSimGetObjectName(simInt handle)
{
  return simGetObjectName(handle);
}

simInt SimInterface::vSimReleaseBuffer(simChar *buffer)
{
  return simReleaseBuffer(buffer);
}

simInt SimInterface::vSimRemoveModel(simInt handle)
{
  return simRemoveModel(handle);
}

simInt SimInterface::vSimSetObjectPosition(simInt handle, simInt relative_to_handle, const simFloat *position)
{
  return simSetObjectPosition(handle, relative_to_handle, position);
}

simInt SimInterface::vSimSetObjectOrientation(simInt handle, simInt relative_to_handle, const simFloat *orientation)
{
  return simSetObjectOrientation(handle, relative_to_handle, orientation);
}

simInt SimInterface::vSimGetObjectPosition(simInt handle, simInt relative_to_handle, simFloat *position)
{
  return simGetObjectPosition(handle, relative_to_handle, position);
}

simInt SimInterface::vSimGetObjectOrientation(simInt handle, simInt relative_to_handle, simFloat *orientation)
{
  return simGetObjectOrientation(handle, relative_to_handle, orientation);
}

