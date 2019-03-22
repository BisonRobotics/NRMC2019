#ifndef STEPPER_TYPES_H
#define STEPPER_TYPES_H

namespace stepper
{
  enum MessageType
  {
    Scan,
    RequestState,
    FindZero,
    SetZero,
    SetMode,
    SetLimits,
    SetPoint,
    StateMessage,
    Error
  };

  enum Mode
  {
    Disabled,
    Position,
    Velocity,
    ControlLoop
  };
}

#endif //STEPPER_TYPES_H
