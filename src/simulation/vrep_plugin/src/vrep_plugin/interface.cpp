#include <vrep_plugin/interface.h>
#include <vrep_library/v_repConst.h>
#include <vrep_plugin/exceptions.h>
#include <vrep_library/v_repLib.h>
#include <ros/ros.h>

using namespace vrep_plugin;

Interface::Interface() : last_error_mode(0) {}

void Interface::setJointPosition(int object_handle, double position)
{
  if (vSimSetJointPosition((simInt) object_handle, (simFloat) position) == -1)
  {
    throw vrep_error("[Interface::setPosition]: Unable to set position for object: "
                     + std::to_string(object_handle));
  }
}

void Interface::setEffort(int object_handle, double effort)
{
  throw vrep_error("[Interface::setEffort]: Not implemented");
}

void Interface::setVelocity(int object_handle, double velocity)
{
  if (vSimSetJointTargetVelocity((simInt) object_handle, (simFloat) velocity) == -1)
  {
    throw vrep_error("[Interface::setVelocity]: Unable to set velocity for object: "
                     + std::to_string(object_handle));
  }
}

double Interface::getPosition(int object_handle)
{
  simFloat position;
  if (vSimGetJointPosition((simInt) object_handle, &position) == -1)
  {
    throw vrep_error("[Interface::getPosition]: Unable to get position for object: "
                     + std::to_string(object_handle));
  }
  return position;
}

double Interface::getEffort(int object_handle)
{
  simFloat effort;
  if (vSimGetJointForce((simInt) object_handle, &effort) == -1)
  {
    throw vrep_error("[Interface::getEffort]: Unable to get effort for object: "
                     + std::to_string(object_handle));
  }
  return effort;
}

double Interface::getVelocity(int object_handle)
{
  simFloat velocity;
  if (vSimGetObjectFloatParameter(object_handle, sim_jointfloatparam_velocity, &velocity) == -1)
  {
    throw vrep_error("[Interface::getVelocity]: Unable to get velocity for object: "
                     + std::to_string(object_handle));
  }
  return velocity;
}

int Interface::getObjectHandle(const std::string &object_name)
{
  int object_handle = vSimGetObjectHandle(object_name.c_str());
  if (object_handle == -1)
  {
    throw vrep_error("[Interface::getObjectHandle]: "
                     "Unable to find object handle for: " + object_name);
  }
  return object_handle;
}

int Interface::loadScene(const std::string &filename)
{
  int handle = vSimLoadScene(filename.c_str());
  if(handle == -1)
  {
    throw vrep_error("[Interface::loadScene]: Unable to load scene: " + filename);
  }
  return handle;
}

void Interface::disableErrorReporting()
{
  if (vSimGetIntegerParameter(sim_intparam_error_report_mode, &last_error_mode) == -1)
  {
    throw vrep_error("[Interface::disableErrorReporting]: "
                     "Unable to get error report mode");
  }
  if (vSimSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore) == -1)
  {
    throw vrep_error("[Interface::disableErrorReporting]: "
                     "Unable to set error report mode");
  }
}

void Interface::resumeErrorReporting()
{
  if (vSimSetIntegerParameter(sim_intparam_error_report_mode, last_error_mode) == -1)
  {
    throw vrep_error("[Interface::resumeErrorReporting]: "
                     "Unable to set error report mode");
  }
}

rosgraph_msgs::Clock Interface::getSimulationTime()
{
  simFloat sim_time_f = vSimGetSimulationTime();
  if (sim_time_f < 0.0f)
  {
    throw vrep_error("[Interface::getSimulationTime]: "
                     "Unable to get simulation time");
  }
  ros::Time sim_time(sim_time_f);
  rosgraph_msgs::Clock sim_time_msg;
  sim_time_msg.clock.nsec = sim_time.nsec;
  sim_time_msg.clock.sec = sim_time.sec;
  return sim_time_msg;
}

void Interface::info(const std::string &message)
{
  ROS_INFO("%s", message.c_str());
  if (vSimAddStatusBarMessage(("[INFO]: " + message).c_str()) == -1)
  {
    throw vrep_error("[Interface::info]: "
                     "Unable add a status bar message");
  }
}

void Interface::warn(const std::string &message)
{
  ROS_WARN("%s", message.c_str());
  if (vSimAddStatusBarMessage(("[WARN]: " + message).c_str()) == -1)
  {
    throw vrep_error("[Interface::warn]: "
                     "Unable add a status bar message");
  }
}

void Interface::error(const std::string &message)
{
  ROS_ERROR("%s", message.c_str());
  if (vSimAddStatusBarMessage(("[ERROR]: " + message).c_str()) == -1)
  {
    throw vrep_error("[Interface::error]: "
                     "Unable add a status bar message");
  }
}

bool Interface::isHandleValid(int object_handle, int object_type)
{
  return (bool)vSimIsHandleValid(object_handle, object_type);
}

int Interface::loadModel(const std::string &filename)
{
  int handle = vSimLoadModel(filename.c_str());
  if (handle == -1)
  {
    throw vrep_error("[Interface::loadModel]: Unable to load model: " + filename);
  }
  return handle;
}

int Interface::findObjectInTree(int base_handle, const std::string &object_name, int object_type)
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
    throw vrep_error("[Interface::findObjectInTree]: Unable to find object: " +
                     object_name);
  }
  if (vSimReleaseBuffer((simChar*)tree_handles) == -1)
  {
    throw vrep_error("[Interface::findObjectInTree]: Unable to release buffer");
  }

  return handle;
}

void Interface::removeModel(int object_handle)
{
  if (vSimRemoveModel(object_handle) == -1)
  {
    throw vrep_error("[Interface::removeModel]: Unable to remove model: " +
                     std::to_string(object_handle));
  }
}

void Interface::setObjectPosition(int handle, int relative_to_handle,
    double x, double y, double z)
{
  simFloat position[]{(simFloat)x, (simFloat)y, (simFloat)z};
  if (vSimSetObjectPosition((simInt)handle, (simInt)relative_to_handle, position) == -1)
  {
    throw vrep_error("[Interface::setObjectPosition]: "
                     "Unable to set object position: " + std::to_string(handle));
  }
}

void Interface::setObjectOrientation(int handle, int relative_to_handle, double alpha, double beta, double gamma)
{
  simFloat orientation[]{(simFloat)alpha, (simFloat)beta, (simFloat)gamma};
  if (vSimSetObjectOrientation((simInt)handle, (simInt)relative_to_handle, orientation) == -1)
  {
    throw vrep_error("[Interface::setObjectOrientation]: "
                     "Unable to set object orientation: " + std::to_string(handle));
  }
}

tuple3d Interface::getObjectPosition(int handle, int relative_to_handle)
{
  simFloat position[3];
  if (vSimGetObjectPosition((simInt)handle, (simInt)relative_to_handle, position) == -1)
  {
    throw vrep_error("[Interface::getObjectPosition]: "
                     "Unable to get object position");
  }
  return std::make_tuple((double)position[0], (double)position[1], (double)position[2]);
}

tuple3d Interface::getObjectOrientation(int handle, int relative_to_handle)
{
  simFloat orientation[3];
  if (vSimGetObjectPosition((simInt)handle, (simInt)relative_to_handle, orientation) == -1)
  {
    throw vrep_error("[Interface::getObjectOrientation]: "
                     "Unable to get object orientation");
  }
  return std::make_tuple((double)orientation[0], (double)orientation[1], (double)orientation[2]);
}

tuple3d Interface::getObjectSize(int handle)
{
  simFloat size[3];
  if (vSimGetObjectSizeValues(handle, (simFloat*) &size) == -1)
  {
    throw vrep_error("[Interface::getObjectSize]: "
                     "Unable to get object size");
  }
  return std::make_tuple((double)size[0], (double)size[1], (double)size[2]);
}

double Interface::getFloatParameter(int handle, int parameter_id)
{
  simFloat parameter;
  if (vSimGetObjectFloatParameter((simInt)handle, (simInt)parameter_id, &parameter) == -1)
  {
    info("[Interface::getFloatParameter]: "
         "Unable to get parameter: " + std::to_string(parameter_id));
    throw vrep_error("[Interface::getFloatParameter]: "
                     "Unable to get parameter: " + std::to_string(parameter_id));
  }
  return (double)parameter;
}

void Interface::setParameter(int handle, int parameter_id, double value)
{
  if (vSimSetObjectFloatParameter((simInt) handle, (simInt) parameter_id, (simFloat) value) == -1)
  {
    throw vrep_error("[Interface::setParameter]: "
                     "Unable to set parameter: " + std::to_string(parameter_id) +
                     " to: " + std::to_string(value));
  }
}

void Interface::setParameter(int parameter_id, bool value)
{
  if (vSimSetBoolParameter((simInt) parameter_id, (simBool) value) == -1)
  {
    throw vrep_error("[Interface::setParameter]: "
                     "Unable to set parameter: " + std::to_string(parameter_id) +
                     " to: " + std::to_string(value));
  }
}

void Interface::startSimulation()
{
  if (vSimStartSimulation() == -1)
  {
    throw vrep_error("[Interface::startSimulation]: Unable to start");
  }
}

void Interface::pauseSimulation()
{
  if (vSimPauseSimulation() == -1)
  {
    throw vrep_error("[Interface::pauseSimulation]: Unable to pause");
  }
}

void Interface::stopSimulation()
{
  if (vSimStopSimulation() == -1)
  {
    throw vrep_error("[Interface::stopSimulation]: Unable to stop");
  }
}

void Interface::shutdown()
{
  vSimQuitSimulator(0);
}


simInt Interface::vSimAddStatusBarMessage(const simChar *message)
{
  return simAddStatusbarMessage(message);
}

simInt Interface::vSimGetObjectHandle(const simChar *object_name)
{
  return simGetObjectHandle(object_name);
}

simInt Interface::vSimGetJointPosition(simInt object_handle, simFloat *position)
{
  return simGetJointPosition(object_handle, position);
}

simInt Interface::vSimSetJointPosition(simInt object_handle, simFloat position)
{
  return simSetJointPosition(object_handle, position);
}

simInt Interface::vSimGetObjectFloatParameter(simInt object_handle, simInt parameter_id, simFloat *parameter)
{
  return simGetObjectFloatParameter(object_handle, parameter_id, parameter);
}

simInt Interface::vSimSetObjectFloatParameter(simInt handle, simInt parameter_id, simFloat parameter)
{
  return simSetObjectFloatParameter(handle, parameter_id, parameter);
}

simInt Interface::vSimGetJointForce(simInt object_handle, simFloat *force)
{
  return simGetJointForce(object_handle, force);
}

simInt Interface::vSimSetJointTargetVelocity(simInt object_handle, simFloat velocity)
{
  return simSetJointTargetVelocity(object_handle, velocity);
}

simInt Interface::vSimLoadScene(const simChar *filename)
{
  return simLoadScene(filename);
}

simInt Interface::vSimSetIntegerParameter(simInt parameter, simInt int_state)
{
  return simSetIntegerParameter(parameter, int_state);
}

simInt Interface::vSimGetIntegerParameter(simInt parameter, simInt *int_state)
{
  return simGetIntegerParameter(parameter, int_state);
}

simFloat Interface::vSimGetSimulationTime()
{
  return simGetSimulationTime();
}

simInt Interface::vSimIsHandleValid(simInt object_handle, simInt object_type)
{
  return simIsHandleValid(object_handle, object_type);
}

simInt Interface::vSimLoadModel(const simChar *filename)
{
  return simLoadModel(filename);
}

simInt* Interface::vSimGetObjectsInTree(simInt handle, simInt type, simInt options, simInt *count)
{
  return simGetObjectsInTree(handle, type, options, count);
}

simChar* Interface::vSimGetObjectName(simInt handle)
{
  return simGetObjectName(handle);
}

simInt Interface::vSimReleaseBuffer(simChar *buffer)
{
  return simReleaseBuffer(buffer);
}

simInt Interface::vSimRemoveModel(simInt handle)
{
  return simRemoveModel(handle);
}

simInt Interface::vSimSetObjectPosition(simInt handle, simInt relative_to_handle, const simFloat *position)
{
  return simSetObjectPosition(handle, relative_to_handle, position);
}

simInt Interface::vSimSetObjectOrientation(simInt handle, simInt relative_to_handle, const simFloat *orientation)
{
  return simSetObjectOrientation(handle, relative_to_handle, orientation);
}

simInt Interface::vSimGetObjectPosition(simInt handle, simInt relative_to_handle, simFloat *position)
{
  return simGetObjectPosition(handle, relative_to_handle, position);
}

simInt Interface::vSimGetObjectOrientation(simInt handle, simInt relative_to_handle, simFloat *orientation)
{
  return simGetObjectOrientation(handle, relative_to_handle, orientation);
}

simInt Interface::vSimGetObjectSizeValues(simInt handle, simFloat *size_values)
{
  return simGetObjectSizeValues(handle, size_values);
}

simInt Interface::vSimStartSimulation()
{
  return simStartSimulation();
}

simInt Interface::vSimPauseSimulation()
{
  return simPauseSimulation();
}

simInt Interface::vSimStopSimulation()
{
  return simStopSimulation();
}

void Interface::vSimQuitSimulator(simBool do_not_display_messages)
{
  return simQuitSimulator(0);
}

simInt Interface::vSimSetBoolParameter(simInt parameter, simBool state)
{
  return simSetBoolParameter(parameter, state);
}

