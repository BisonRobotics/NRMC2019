#ifndef VREP_PLUGIN_INTERFACE_H
#define VREP_PLUGIN_INTERFACE_H

#include <vrep_library/v_repTypes.h>
#include <string>
#include <tuple>
#include <Eigen/Core>
#include <ros/ros.h>

namespace vrep_plugin
{

typedef std::tuple<double, double, double> tuple3d;

class Interface
{
  public:
    Interface();

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
    ros::Time getSimulationTime();
    int findObjectInTree(int base_handle, const std::string &name, int type);
    void setObjectPosition(int handle, int relative_to_handle,
        double x, double y, double z);
    void setObjectOrientation(int handle, int relative_to_handle, double alpha,
                              double beta, double gamma);
    tuple3d getObjectPosition(int handle, int relative_to_handle);
    tuple3d getObjectOrientation(int handle, int relative_to_handle);
    tuple3d getObjectSize(int handle);
    std::pair<tuple3d, tuple3d> readForceSensor(int handle);
    double getMass(int handle);
    Eigen::Matrix<double, 4, 4> getObjectMatrix(int handle, int relative_to_handle = -1);
    double getFloatParameter(int handle, int parameter_id);
    void setParameter(int handle, int parameter_id, double value);
    void setParameter(int parameter_id, bool value);
    void startSimulation();
    void pauseSimulation();
    void stopSimulation();
    void shutdown();

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
    simInt vSimStartSimulation();
    simInt vSimPauseSimulation();
    simInt vSimStopSimulation();
    void vSimQuitSimulator(simBool do_not_display_messages);
    simInt vSimSetBoolParameter(simInt parameter, simBool state);
    simInt vSimReadForceSensor(simInt handle, simFloat *force, simFloat *torque);
    simInt vSimGetShapeMassAndInertia(simInt handle, simFloat *mass,
        simFloat *inertiaMatrix, simFloat *center_of_mass, simFloat *transformation);
    simInt vSimGetObjectMatrix(simInt handle, simInt relative_to_handle, simFloat *object_matrix);

};

}

#endif //VREP_PLUGIN_INTERFACE_H
