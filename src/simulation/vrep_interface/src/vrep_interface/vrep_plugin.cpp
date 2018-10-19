#include "vrep_library/v_repLib.h"
#include "vrep_interface/vrep_plugin.h"
#include "vrep_interface/ros_server.h"

#include "ros/ros.h"
#include <iostream>
#include <vrep_interface/vrep_interface.h>

#define PLUGIN_VERSION 1

LIBRARY vrepLib;

// Try to initialize vrep_interface plugin
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt)
{

  char curDirAndFile[1024];
  getcwd(curDirAndFile, sizeof(curDirAndFile));

  std::string currentDirAndPath(curDirAndFile);
  std::string temp(currentDirAndPath);
  temp += "/libv_rep.so";

  vrepLib = loadVrepLibrary(temp.c_str());
  if (vrepLib == NULL)
  {
    std::cout << "Error, could not find or correctly load the V-REP library. "
                 "Cannot start 'vrep_interface' plugin.\n";
    return (0); // Error
  }
  if (getVrepProcAddresses(vrepLib) == 0)
  {
    std::cout << "Error, could not find all required functions in the V-REP "
                 "library. Cannot start 'vrep_interface' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return (0); // Error
  }

  int vrepVer;
  simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
  if (vrepVer < 30102)
  {
    std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start "
                 "'vrep_interface' plugin.\n";
    unloadVrepLibrary(vrepLib);
    return (0); // Error
  }

  if (!vrep_interface::VREPInterface::initialize())
  {
    std::cout << "ROS master is not running. Cannot start 'vrep_interface' plugin.\n";
    return (0);  // Error
  }

  return (PLUGIN_VERSION); // Success
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e.
// releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
  vrep_interface::VREPInterface::shutDown();
  unloadVrepLibrary(vrepLib);
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
    vrep_interface::VREPInterface::instancePass();
  }

  // Main script is about to be run (only called while a simulation is running
  // (and not paused!))
  //
  // This is a good location to execute simulation commands
  if (message == sim_message_eventcallback_mainscriptabouttobecalled)
  {
    vrep_interface::VREPInterface::mainScriptAboutToBeCalled();
  }

  // Simulation is about to start
  if (message == sim_message_eventcallback_simulationabouttostart)
  {
    vrep_interface::VREPInterface::simulationAboutToStart();
  }

  // Simulation just ended
  if (message == sim_message_eventcallback_simulationended)
  {
    vrep_interface::VREPInterface::simulationEnded();
  }

  // Keep following unchanged:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);  // restore previous settings
  return (retVal);
}
