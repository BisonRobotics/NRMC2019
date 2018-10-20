#ifndef VREP_INTERFACE_INTERFACE_H
#define VREP_INTERFACE_INTERFACE_H

#include <vrep_library/v_repTypes.h>
#include <string>
#include <rosgraph_msgs/Clock.h>
#include <tuple>

namespace vrep_interface
{

typedef std::tuple<double, double, double> tuple3d;

class SimInterface
{
  public:
    void setJointPosition(int handle, double position);
    void setEffort(int handle, double effort);
    void setVelocity(int handle, double velocity);
    double getPosition(int handle);
    double getEffort(int handle);
    double getVelocity(int handle);
    int getObjectHandle(const std::string &name);
    bool isHandleValid(int handle, int type);
    int loadScene(std::string filename);
    int loadModel(std::string filename);
    void removeModel(int handle);
    void disableErrorReporting();
    void resumeErrorReporting();
    rosgraph_msgs::Clock getSimulationTime();
    int findObjectInTree(int base_handle, const std::string &name, int type);
    void setObjectPosition(int handle, int relative_to_handle,
        double x, double y, double z);
    void setObjectOrientation(int handle, int relative_to_handle, double alpha,
                              double beta, double gamma);
    tuple3d getObjectPosition(int handle, int relative_to_handle);
    tuple3d getObjectOrientation(int handle, int relative_to_handle);

    void info(const std::string &message);
    void warn(const std::string &message);
    void error(const std::string &message);

  protected:
    simInt last_error_mode;

    virtual simInt vSimAddStatusBarMessage(const simChar *message) = 0;
    virtual simInt vSimGetObjectHandle(const simChar *name) = 0;
    virtual simInt vSimIsHandleValid(simInt handle, simInt type) = 0;
    virtual simInt vSimGetJointPosition(simInt handle, simFloat *position) = 0;
    virtual simInt vSimSetJointPosition(simInt handle, simFloat position) = 0;
    virtual simInt vSimGetObjectFloatParameter(simInt handle, simInt parameter_id,
        simFloat *parameter) = 0;
    virtual simInt vSimGetJointForce(simInt handle, simFloat *force) = 0;
    virtual simInt vSimSetJointTargetVelocity(simInt handle, simFloat velocity) = 0;
    virtual simInt vSimLoadScene(std::string filename) = 0;
    virtual simInt vSimLoadModel(std::string filename) = 0;
    virtual simInt vSimSetIntegerParameter(simInt parameter, simInt int_state) = 0;
    virtual simInt vSimGetIntegerParameter(simInt parameter, simInt *int_state) = 0;
    virtual simFloat vSimGetSimulationTime() = 0;
    virtual simInt* vSimGetObjectsInTree(simInt handle, simInt type,
        simInt options, simInt *count) = 0;
    virtual simChar* vSimGetObjectName(simInt handle) = 0;
    virtual simInt vSimReleaseBuffer(simChar *buffer) = 0;
    virtual simInt vSimRemoveModel(simInt handle) = 0;
    virtual simInt vSimSetObjectPosition(simInt handle,
        simInt relative_to_handle, const simFloat *position) = 0;
    virtual simInt vSimSetObjectOrientation(simInt handle,
         simInt relative_to_handle, const simFloat *orientation) = 0;
    virtual simInt vSimGetObjectPosition(simInt handle, simInt relative_to_handle,
        simFloat *position) = 0;
    virtual simInt vSimGetObjectOrientation(simInt handle, simInt relative_to_handle,
        simFloat *orientation) = 0;
};

}

#endif //VREP_INTERFACE_INTERFACE_H
