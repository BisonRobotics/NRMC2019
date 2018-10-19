#ifndef VREP_INTERFACE_VREP_INTERFACE_H
#define VREP_INTERFACE_VREP_INTERFACE_H

#include <vrep_interface/ros_server.h>

namespace vrep_interface
{
class VREPInterface
{
  public:
    static bool initialize();
    static void shutDown();
    static void instancePass();
    static void mainScriptAboutToBeCalled();
    static void simulationAboutToStart();
    static void simulationEnded();

  private:
    static ROSServer *server;
};
}

#endif //VREP_INTERFACE_VREP_INTERFACE_H
