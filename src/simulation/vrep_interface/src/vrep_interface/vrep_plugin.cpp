#include "vrep_library/v_repLib.h"
#include "vrep_interface/vrep_plugin.h"

#include "ros/ros.h"
#include <iostream>

#define PLUGIN_VERSION 1

LIBRARY vrepLib;  // the V-REP library that we will dynamically load and bind

// This is the plugin start routine (called just once, just after the plugin was
// loaded):
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt)
{
  // Dynamically load and bind V-REP functions:
  // ******************************************
  // 1. Figure out this plugin's directory:
  char curDirAndFile[1024];
  getcwd(curDirAndFile, sizeof(curDirAndFile));

  std::string currentDirAndPath(curDirAndFile);
  // 2. Append the V-REP library's name:
  std::string temp(currentDirAndPath);
#ifdef _WIN32
  temp += "\\v_rep.dll";
#elif defined(__linux)
  temp += "/libv_rep.so";
#elif defined(__APPLE__)
  temp += "/libv_rep.dylib";
#endif

  // 3. Load the V-REP library:
  vrepLib = loadVrepLibrary(temp.c_str());
  if (vrepLib == NULL)
  {
    std::cout << "Error, could not find or correctly load the V-REP library. "
                 "Cannot start 'rosSkeleton' plugin.\n";
    return (0);  // Means error, V-REP will unload this plugin
  }
  if (getVrepProcAddresses(vrepLib) == 0)
  {
    std::cout << "Error, could not find all required functions in the V-REP "
                 "library. Cannot start 'rosSkeleton' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return (0);  // Means error, V-REP will unload this plugin
  }
  // ******************************************

  // Check the version of V-REP:
  // ******************************************
  int vrepVer;
  simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
  if (vrepVer < 30102)  // if V-REP version is smaller than 3.01.02
  {
    std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start "
                 "'rosSkeleton' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return (0);  // Means error, V-REP will unload this plugin
  }
  // ******************************************

  // Initialize the ROS part:
  if (!vrep_interface::VREPPlugin::initialize())
  {
    std::cout << "ROS master is not running. Cannot start 'rosSkeleton' plugin.\n";
    return (0);  // If the master is not running then the plugin is not loaded.
  }

  return (PLUGIN_VERSION);  // initialization went fine, we return the version
                            // number of this plugin (can be queried with
                            // simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e.
// releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
  vrep_interface::VREPPlugin::shutDown();      // shutdown the vrep_interface_server
  unloadVrepLibrary(vrepLib);  // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very
// often, with various messages):
VREP_DLLEXPORT void *v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData)
{
  // This is called quite often. Just watch out for messages/events you want to
  // handle
  // Keep following 4 lines at the beginning and unchanged:
  int errorModeSaved;
  simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
  simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
  void *retVal = NULL;

  // Here we can intercept many messages from V-REP (actually callbacks). Only
  // the most important messages are listed here:

  // This message is sent each time the scene was rendered (well, shortly
  // after) (very often)
  // When a simulation is not running, but you still need to execute some
  // commands, then put some code here
  if (message == sim_message_eventcallback_instancepass)
  {
    vrep_interface::VREPPlugin::instancePass();
  }

  // Main script is about to be run (only called while a simulation is running
  // (and not paused!))
  //
  // This is a good location to execute simulation commands
  if (message == sim_message_eventcallback_mainscriptabouttobecalled)
  {
    vrep_interface::VREPPlugin::mainScriptAboutToBeCalled();
  }

  // Simulation is about to start
  if (message == sim_message_eventcallback_simulationabouttostart)
  {
    vrep_interface::VREPPlugin::simulationAboutToStart();
  }

  // Simulation just ended
  if (message == sim_message_eventcallback_simulationended)
  {
    vrep_interface::VREPPlugin::simulationEnded();
  }

  // Keep following unchanged:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);  // restore previous settings
  return (retVal);
}

using namespace vrep_interface;

VREPServer *VREPPlugin::server = NULL;
VREPInterface *VREPPlugin::sim_interface = NULL;

bool VREPPlugin::initialize()
{
  try
  {
    sim_interface = new VREPInterface;
    server = new VREPServer(sim_interface);
  }
  catch (std::exception &e)
  {
    return false;
  }
  return true;
}

void VREPPlugin::shutDown()
{
  delete server;
  delete sim_interface;
}

void VREPPlugin::instancePass() // Simulation not running
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    server->spinOnce();
  }
}

void VREPPlugin::mainScriptAboutToBeCalled() // Simulation running
{
  server->spinOnce();
}

void VREPPlugin::simulationAboutToStart()
{
  info("Starting simulation");
  server->simulationAboutToStart();
}

void VREPPlugin::simulationEnded()
{
  info("Simulation ended");
  server->simulationEnded();
}

void VREPPlugin::info(const std::string &message)
{
  simAddStatusbarMessage(("[INFO]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}

void VREPPlugin::warn(const std::string &message)
{
  simAddStatusbarMessage(("[WARN]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}

void VREPPlugin::error(const std::string &message)
{
  simAddStatusbarMessage(("[ERROR]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}
