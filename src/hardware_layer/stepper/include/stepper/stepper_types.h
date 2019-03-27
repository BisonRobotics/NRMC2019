#ifndef STEPPER_TYPES_H
#define STEPPER_TYPES_H

namespace stepper
{
  enum MessageType
  {
    Error        = 0,
    RequestState = 1,
    FindZero     = 2,
    SetZero      = 3,
    SetMode      = 4,
    SetLimits    = 5,
    SetPoint     = 6,
    StateMessage = 7, // position & velocity
  };

  enum Mode
  {
    Disabled    = 0,
    Initialize  = 1,
    Scan        = 2,
    Position    = 3,
    Velocity    = 4,
    ControlLoop = 5
  };
}

#endif //STEPPER_TYPES_H
