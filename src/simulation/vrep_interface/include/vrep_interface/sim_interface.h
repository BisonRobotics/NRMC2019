#ifndef VREP_INTERFACE_INTERFACE_H
#define VREP_INTERFACE_INTERFACE_H

#include <vrep_library/v_repTypes.h>
#include <string>
#include <rosgraph_msgs/Clock.h>

namespace vrep_interface
{

class SimInterface
{
  public:
    void setPosition(int object_handle, double position);
    void setEffort(int object_handle, double effort);
    void setVelocity(int object_handle, double velocity);
    double getPosition(int object_handle);
    double getEffort(int object_handle);
    double getVelocity(int object_handle);
    void loadScene(const std::string &filename);
    void turnOffErrorReporting();
    void turnOnErrorReporting();
    rosgraph_msgs::Clock getSimulationTime();

    static void info(const std::string &message){};
    static void warn(const std::string &message) {};
    static void error(const std::string &message) {};

  protected:
    simInt last_error_mode;

    virtual simInt vSimAddStatusBarMessage(const simChar *message) = 0;
    virtual simInt vSimGetObjectHandle(const simChar *object_name) = 0;
    virtual simInt vSimGetJointPosition(simInt object_handle, simFloat *position) = 0;
    virtual simInt vSimSetJointPosition(simInt object_handle, simFloat position) = 0;
    virtual simInt vSimGetObjectFloatParameter(simInt object_handle, simInt parameter_id, simFloat *parameter) = 0;
    virtual simInt vSimGetJointForce(simInt object_handle, simFloat *force) = 0;
    virtual simInt vSimSetJointTargetVelocity(simInt object_handle, simFloat velocity) = 0;
    virtual simInt vSimLoadScene(const simChar *filename) = 0;
    virtual simInt vSimSetIntegerParameter(simInt parameter, simInt int_state) = 0;
    virtual simInt vSimGetIntegerParameter(simInt parameter, simInt *int_state) = 0;
    virtual simFloat vSimGetSimulationTime() = 0;
};

}

#endif //VREP_INTERFACE_INTERFACE_H
