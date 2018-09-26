// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April
// 5th 2017

#include "vrep_lib/v_repLib.h"
#include "vrep_interface_plugin/vrep_interface_plugin.h"
#include "ros_interface_server/ros_interface_server.h"

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
  if (!vrep_interface::ros_server::initialize())
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
  vrep_interface::ros_server::shutDown();      // shutdown the ros_server
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
    vrep_interface::ros_server::instancePass();
  }

  // Main script is about to be run (only called while a simulation is running
  // (and not paused!))
  //
  // This is a good location to execute simulation commands
  if (message == sim_message_eventcallback_mainscriptabouttobecalled)
  {
    vrep_interface::ros_server::mainScriptAboutToBeCalled();
  }

  // Simulation is about to start
  if (message == sim_message_eventcallback_simulationabouttostart)
  {
    vrep_interface::ros_server::simulationAboutToStart();
  }

  // Simulation just ended
  if (message == sim_message_eventcallback_simulationended)
  {
    vrep_interface::ros_server::simulationEnded();
  }

  // Keep following unchanged:
  simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved);  // restore previous settings
  return (retVal);
}
