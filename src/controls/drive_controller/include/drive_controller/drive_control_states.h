#ifndef DRIVE_CONTROLLER_DRIVE_CONTROL_STATES_H
#define DRIVE_CONTROLLER_DRIVE_CONTROL_STATES_H

#include <drive_controller/DriveControlAction.h>
#include <string>

namespace drive_controller
{
  enum class ControlState
  {
    error = 0,
    ready,
    new_goal,
    in_progress,
    cancel,
    manual
  };

  DriveControlResult toResult(ControlState state);
  ControlState toControlState(DriveControlGoal goal);
  std::string to_string(ControlState state);
}

#endif //DRIVE_CONTROLLER_DRIVE_CONTROL_STATES_H
