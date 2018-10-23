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
    SimInterface();

    void setJointPosition(int handle, double position);
    void setEffort(int handle, double effort);
    void setVelocity(int handle, double velocity);
    double getPosition(int handle);
    double getEffort(int handle);
    double getVelocity(int handle);
    int getObjectHandle(const std::string &name);
    bool isHandleValid(int handle, int type);
    int loadScene(const std::string &filename);
    int loadModel(const std::string &filename);
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
    tuple3d getObjectSize(int handle);
    double getFloatParameter(int handle, int parameter_id);
    void setParameter(int handle, int parameter_id, double value);

    void info(const std::string &message);
    void warn(const std::string &message);
    void error(const std::string &message);

  protected:
    simInt last_error_mode;

    simInt vSimAddStatusBarMessage(const simChar *message);
    simInt vSimGetObjectHandle(const simChar *name);
    simInt vSimIsHandleValid(simInt handle, simInt type);
    simInt vSimGetJointPosition(simInt handle, simFloat *position);
    simInt vSimSetJointPosition(simInt handle, simFloat position);
    simInt vSimGetObjectFloatParameter(simInt handle, simInt parameter_id, simFloat *parameter);
    simInt vSimSetObjectFloatParameter(simInt handle, simInt parameter_id, simFloat parameter);
    simInt vSimGetJointForce(simInt handle, simFloat *force);
    simInt vSimSetJointTargetVelocity(simInt handle, simFloat velocity);
    simInt vSimLoadScene(const simChar *filename);
    simInt vSimLoadModel(const simChar *filename);
    simInt vSimSetIntegerParameter(simInt parameter, simInt int_state);
    simInt vSimGetIntegerParameter(simInt parameter, simInt *int_state);
    simFloat vSimGetSimulationTime();
    simInt* vSimGetObjectsInTree(simInt handle, simInt type, simInt options, simInt *count);
    simChar* vSimGetObjectName(simInt handle);
    simInt vSimReleaseBuffer(simChar *buffer);
    simInt vSimRemoveModel(simInt handle);
    simInt vSimSetObjectPosition(simInt handle, simInt relative_to_handle, const simFloat *position);
    simInt vSimSetObjectOrientation(simInt handle, simInt relative_to_handle, const simFloat *orientation);
    simInt vSimGetObjectPosition(simInt handle, simInt relative_to_handle, simFloat *position);
    simInt vSimGetObjectOrientation(simInt handle, simInt relative_to_handle, simFloat *orientation);
    simInt vSimGetObjectSizeValues(simInt handle, simFloat *size_values);
};

}

#endif //VREP_INTERFACE_INTERFACE_H
