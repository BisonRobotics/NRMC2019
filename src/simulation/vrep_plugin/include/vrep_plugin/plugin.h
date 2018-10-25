#ifndef VREP_PLUGIN_PLUGIN_H
#define VREP_PLUGIN_PLUGIN_H

#include <vrep_library/v_repLib.h>
#include <vrep_plugin/server.h>

#define VREP_DLLEXPORT extern "C"
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void *v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData);

namespace vrep_plugin
{
class Plugin
{
  public:
    static bool initialize();
    static void shutDown();
    static void instancePass();
    static void mainScriptAboutToBeCalled();
    static void simulationAboutToStart();
    static void simulationEnded();

  private:
    Plugin(){};
    static Interface *sim_interface;
    static Server *server;

};

}


#endif
