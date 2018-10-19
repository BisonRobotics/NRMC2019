#ifndef VREP_INTERFACE_VREP_PLUGIN_H
#define VREP_INTERFACE_VREP_PLUGIN_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>

#include <vrep_library/v_repLib.h>

#include <vrep_robot/vrep_robot.h>
#include <vrep_msgs/SpawnRobot.h>
#include <vrep_interface/vrep_server.h>
#include <vrep_interface/sim_interface.h>
#include <vrep_interface/vrep_interface.h>

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
    static void info(const std::string &message);
    static void warn(const std::string &message);
    static void error(const std::string &message);

  private:
    VREPPlugin(){};
    static VREPInterface *sim_interface;
    static VREPServer *server;

};

}


#endif
