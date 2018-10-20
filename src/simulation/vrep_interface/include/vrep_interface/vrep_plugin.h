#ifndef VREP_INTERFACE_VREP_PLUGIN_H
#define VREP_INTERFACE_VREP_PLUGIN_H

#include <vrep_library/v_repLib.h>
#include <vrep_interface/vrep_server.h>

// vrep entry points
#define VREP_DLLEXPORT extern "C"
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void *v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData);

namespace vrep_interface
{
class VREPPlugin
{
  public:
    static bool initialize();
    static void shutDown();
    static void instancePass();
    static void mainScriptAboutToBeCalled();
    static void simulationAboutToStart();
    static void simulationEnded();

  private:
    VREPPlugin(){};
    static SimInterface *sim_interface;
    static VREPServer *server;

};

}


#endif
