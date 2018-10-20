#ifndef VREP_INTERFACE_VREP_INTERFACE_H
#define VREP_INTERFACE_VREP_INTERFACE_H

#include <vrep_interface/sim_interface.h>

namespace vrep_interface
{

class VREPInterface : public SimInterface
{
  protected:
    simInt vSimAddStatusBarMessage(const simChar *message) override;
    simInt vSimGetObjectHandle(const simChar *object_name) override;
    simInt vSimIsHandleValid(simInt object_handle, simInt object_type) override;
    simInt vSimGetJointPosition(simInt object_handle, simFloat *position) override;
    simInt vSimSetJointPosition(simInt object_handle, simFloat position) override;
    simInt vSimGetObjectFloatParameter(simInt object_handle, simInt parameter_id,
        simFloat *parameter) override;
    simInt vSimGetJointForce(simInt object_handle, simFloat *force) override;
    simInt vSimSetJointTargetVelocity(simInt object_handle, simFloat velocity) override;
    simInt vSimLoadScene(std::string filename) override;
    simInt vSimLoadModel(std::string filename) override;
    simInt vSimSetIntegerParameter(simInt parameter, simInt int_state) override;
    simInt vSimGetIntegerParameter(simInt parameter, simInt *int_state) override;
    simFloat vSimGetSimulationTime() override;
    simInt* vSimGetObjectsInTree(simInt handle, simInt type, simInt options,
        simInt *count) override;
    simChar* vSimGetObjectName(simInt handle) override;
    simInt vSimReleaseBuffer(simChar *buffer) override;
    simInt vSimRemoveModel(simInt handle) override;
    simInt vSimSetObjectPosition(simInt handle,
        simInt relative_to_handle, const simFloat *position) override;
    simInt vSimSetObjectOrientation(simInt handle,
        simInt relative_to_handle, const simFloat *orientation) override;
    simInt vSimGetObjectPosition(simInt handle, simInt relative_to_handle,
        simFloat *position) override;
    simInt vSimGetObjectOrientation(simInt handle, simInt relative_to_handle,
        simFloat *orientation) override;
};

}

#endif //VREP_INTERFACE_VREP_INTERFACE_H
