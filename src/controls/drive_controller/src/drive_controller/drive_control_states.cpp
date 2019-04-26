#include <drive_controller/drive_control_states.h>

using namespace drive_controller;

std::string drive_controller::to_string(ControlState state)
{
  switch (state)
  {
    case ControlState::error:
      return "error";
    case ControlState::ready:
      return "ready";
    case ControlState::new_goal:
      return "new_goal";
    case ControlState::in_progress:
      return "in_progress";
    case ControlState::cancel:
      return "cancel";
    case ControlState::manual:
      return "manual";
  }
}

DriveControlResult drive_controller::toResult(ControlState state)
{
  DriveControlResult result;
  result.control_state = (DriveControlResult::_control_state_type)state;
  return result;
}

ControlState drive_controller::toControlState(DriveControlGoal goal)
{
  return (ControlState)goal.control_state;
}