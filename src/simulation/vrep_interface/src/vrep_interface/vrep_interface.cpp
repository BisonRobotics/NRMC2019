#include <vrep_interface/vrep_interface.h>


using namespace vrep_interface;

VREPServer *VREPInterface::server = NULL;

bool VREPInterface::initialize()
{
  try
  {
    server = new VREPServer;
  }
  catch (std::exception &e)
  {
    return false;
  }
  return true;
}

void VREPInterface::shutDown()
{
  delete server;
}

void VREPInterface::instancePass() // Simulation not running
{
  if ((simGetSimulationState() & sim_simulation_advancing) == 0)
  {
    server->spinOnce();
  }
}

void VREPInterface::mainScriptAboutToBeCalled() // Simulation running
{
  server->spinOnce();
}

void VREPInterface::simulationAboutToStart()
{
  info("Starting simulation");
  server->simulationAboutToStart();
}

void VREPInterface::simulationEnded()
{
  info("Simulation ended");
  server->simulationEnded();
}

/*******************************************************************************
 * Custom services
 ******************************************************************************/


void VREPInterface::info(const std::string &message)
{
  simAddStatusbarMessage(("[INFO]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}

void VREPInterface::warn(const std::string &message)
{
  simAddStatusbarMessage(("[WARN]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}

void VREPInterface::error(const std::string &message)
{
  simAddStatusbarMessage(("[ERROR]: " + message).c_str());
  std::cout << message.c_str() << std::endl;
}






