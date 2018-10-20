#include <vrep_interface/sim_interface.h>
#include <vrep_library/v_repConst.h>
#include <vrep_exceptions/vrep_exceptions.h>
#include <vrep_library/v_repLib.h>

using namespace vrep_interface;

void SimInterface::setJointPosition(int object_handle, double position)
{
  if (simSetJointPosition((simInt) object_handle, (simFloat) position) == -1)
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
  if (simSetJointTargetVelocity((simInt) object_handle, (simFloat) velocity) == -1)
  {
    throw vrep_error("[SimInterface::setVelocity]: Unable to set velocity for object: "
                     + std::to_string(object_handle));
  }
}

double SimInterface::getPosition(int object_handle)
{
  simFloat position;
  if (simGetJointPosition((simInt) object_handle, &position) == -1)
  {
    throw vrep_error("[SimInterface::getPosition]: Unable to get position for object: "
                     + std::to_string(object_handle));
  }
  return position;
}

double SimInterface::getEffort(int object_handle)
{
  simFloat effort;
  if (simGetJointForce((simInt) object_handle, &effort) == -1)
  {
    throw vrep_error("[SimInterface::getEffort]: Unable to get effort for object: "
                     + std::to_string(object_handle));
  }
  return effort;
}

double SimInterface::getVelocity(int object_handle)
{
  simFloat velocity;
  if (simGetObjectFloatParameter(object_handle, 2012, &velocity) == -1)
  {
    throw vrep_error("[SimInterface::getVelocity]: Unable to get velocity for object: "
                     + std::to_string(object_handle));
  }
  return velocity;
}

int SimInterface::getObjectHandle(const std::string &object_name)
{
  int object_handle = simGetObjectHandle(object_name.c_str());
  if (object_handle == -1)
  {
    throw vrep_error("[SimInterface::getObjectHandle]: "
                     "Unable to find object handle for: " + object_name);
  }
  return object_handle;
}

int SimInterface::loadScene(std::string filename)
{
  std::cout << "Load scene" << std::endl;
  int handle = simLoadScene(filename.c_str());
  std::cout << "Load scene" << std::endl;
  if(handle == -1)
  {
    throw vrep_error("[SimInterface::loadScene]: Unable to load scene: " + filename);
  }
  return handle;
}

void SimInterface::disableErrorReporting()
{
  if (simGetIntegerParameter(sim_intparam_error_report_mode, &last_error_mode) == -1)
  {
    throw vrep_error("[SimInterface::disableErrorReporting]: "
                     "Unable to get error report mode");
  }
  if (simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore) == -1)
  {
    throw vrep_error("[SimInterface::disableErrorReporting]: "
                     "Unable to set error report mode");
  }
}

void SimInterface::resumeErrorReporting()
{
  if (simSetIntegerParameter(sim_intparam_error_report_mode, last_error_mode) == -1)
  {
    throw vrep_error("[SimInterface::resumeErrorReporting]: "
                     "Unable to set error report mode");
  }
}

rosgraph_msgs::Clock SimInterface::getSimulationTime()
{
  simFloat sim_time_f = simGetSimulationTime();
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
  if (simAddStatusbarMessage(("[INFO]: " + message).c_str()) == -1)
  {
    throw vrep_error("[SimInterface::info]: "
                     "Unable add a status bar message");
  }
  std::cout << message.c_str() << std::endl;
}

void SimInterface::warn(const std::string &message)
{
  if (simAddStatusbarMessage(("[WARN]: " + message).c_str()) == -1)
  {
    throw vrep_error("[SimInterface::warn]: "
                     "Unable add a status bar message");
  }
  std::cout << message.c_str() << std::endl;
}

void SimInterface::error(const std::string &message)
{
  if (simAddStatusbarMessage(("[ERROR]: " + message).c_str()) == -1)
  {
    throw vrep_error("[SimInterface::error]: "
                     "Unable add a status bar message");
  }
  std::cout << message.c_str() << std::endl;
}

bool SimInterface::isHandleValid(int object_handle, int object_type)
{
  return (bool)simIsHandleValid(object_handle, object_type);
}

int SimInterface::loadModel(std::string filename)
{
  std::cout << "loadModel" << std::endl;
  int handle = simLoadModel(filename.c_str());
  std::cout << "loadModel" << std::endl;
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
  std::cout << "tree handles" << std::endl;
  tree_handles = simGetObjectsInTree(base_handle, object_type, 0x00, &handle_count);
  std::cout << "tree handles" << std::endl;
  for (int i = 0; i < handle_count; i++)
  {
    std::cout << "name_c" << std::endl;
    simChar *name_c = simGetObjectName(tree_handles[i]);
    std::cout << "name_c" << std::endl;
    if (name_c != NULL)
    {
      std::string name = std::string(name_c);
      if (name == object_name)
      {
        handle = tree_handles[i];
      }
    }
  }
  std::cout << "Here 1" << std::endl;
  if (handle == -1)
  {
    throw vrep_error("[SimInterface::findObjectInTree]: Unable to find object: " +
                     object_name);
  }
  std::cout << "Here 2" << std::endl;
  if (simReleaseBuffer((simChar*)tree_handles) == -1)
  {
    throw vrep_error("[SimInterface::findObjectInTree]: Unable to release buffer");
  }
  std::cout << "Here 3" << std::endl;


  return handle;
}

void SimInterface::removeModel(int object_handle)
{
  if (simRemoveModel(object_handle) == -1)
  {
    throw vrep_error("[SimInterface::removeModel]: Unable to remove model: " +
                     std::to_string(object_handle));
  }
}

void SimInterface::setObjectPosition(int handle, int relative_to_handle,
    double x, double y, double z)
{
  simFloat position[]{(simFloat)x, (simFloat)y, (simFloat)z};
  if (simSetObjectPosition((simInt)handle, (simInt)relative_to_handle, position) == -1)
  {
    throw vrep_error("[SimInterface::setObjectPosition]: "
                     "Unable to set object position: " + std::to_string(handle));
  }
}

void SimInterface::setObjectOrientation(int handle, int relative_to_handle, double alpha, double beta, double gamma)
{
  simFloat orientation[]{(simFloat)alpha, (simFloat)beta, (simFloat)gamma};
  if (simSetObjectOrientation((simInt)handle, (simInt)relative_to_handle, orientation) == -1)
  {
    throw vrep_error("[SimInterface::setObjectOrientation]: "
                     "Unable to set object orientation: " + std::to_string(handle));
  }
}

tuple3d SimInterface::getObjectPosition(int handle, int relative_to_handle)
{
  simFloat position[3];
  if (simGetObjectPosition((simInt)handle, (simInt)relative_to_handle, position) == -1)
  {
    throw vrep_error("[SimInterface::getObjectPosition]: "
                     "Unable to get object position");
  }
  return std::make_tuple((double)position[0], (double)position[1], (double)position[2]);
}

tuple3d SimInterface::getObjectOrientation(int handle, int relative_to_handle)
{
  simFloat orientation[3];
  if (simGetObjectPosition((simInt)handle, (simInt)relative_to_handle, orientation) == -1)
  {
    throw vrep_error("[SimInterface::getObjectOrientation]: "
                     "Unable to get object orientation");
  }
  return std::make_tuple((double)orientation[0], (double)orientation[1], (double)orientation[2]);
}
