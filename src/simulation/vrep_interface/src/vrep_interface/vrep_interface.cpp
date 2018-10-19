#include <vrep_interface/vrep_interface.h>

using namespace vrep_interface;

//ROSServer *server = NULL;

bool VREPInterface::initialize()
{
  //return server->initialize();
  return true;
}

void VREPInterface::shutDown()
{
  //delete server;
}

void VREPInterface::instancePass()
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    //server->spinOnce();
  }
}

void VREPInterface::mainScriptAboutToBeCalled()
{
  //server->spinOnce();
}

void VREPInterface::simulationAboutToStart()
{
  //server->simulationAboutToStart();
}

void VREPInterface::simulationEnded()
{
  //server->simulationEnded();
}
