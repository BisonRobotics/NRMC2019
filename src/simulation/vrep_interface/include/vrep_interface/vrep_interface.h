#ifndef VREP_INTERFACE_VREP_INTERFACE_H
#define VREP_INTERFACE_VREP_INTERFACE_H

#include <vrep_interface/sim_interface.h>

namespace vrep_interface
{

class VREPInterface : public SimInterface
{
  protected:
    simInt vSimAddStatusBarMessage(const simChar *message);
    simInt vSimGetObjectHandle(const simChar *object_name);
    simInt vSimGetJointPosition(simInt object_handle, simFloat *position);
    simInt vSimSetJointPosition(simInt object_handle, simFloat position);
    simInt vSimGetObjectFloatParameter(simInt object_handle, simInt parameter_id, simFloat *parameter);
    simInt vSimGetJointForce(simInt object_handle, simFloat *force);
    simInt vSimSetJointTargetVelocity(simInt object_handle, simFloat velocity);
    simInt vSimLoadScene(const simChar *filename);
    simInt vSimSetIntegerParameter(simInt parameter, simInt int_state);
    simInt vSimGetIntegerParameter(simInt parameter, simInt *int_state);
    simFloat vSimGetSimulationTime();
};

}

#endif //VREP_INTERFACE_VREP_INTERFACE_H
