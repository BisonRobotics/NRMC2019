#include <competition/competition_config.h>

using namespace competition;

std::string competition::to_string(competition::ControlState state)
{
  switch (state)
  {
    case ControlState::manual:
      return "manual";
    case ControlState::assisted_autonomy:
      return "assisted_autonomy";
    case ControlState::wait_for_start:
      return "wait_for_start";
    case ControlState::start:
      return "start";
    case ControlState::check_for_apriltag:
      return "check_for_apriltag";
    case ControlState::wait_for_localization:
      return "wait_for_localization";
    case ControlState::navigate_to_dig_zone_1:
      return "navigate_to_dig_zone_1";
    case ControlState::dig_1:
      return "dig_1";
    case ControlState::finish_dig_1:
      return "finish_dig_1";
    case ControlState::navigate_to_hopper_1:
      return  "navigate_to_hopper_1";
    case ControlState::dump_1:
      return "dump_1";
    case ControlState::navigate_to_dig_zone_2:
      return "navigate_to_dig_zone_2";
    case ControlState::dig_2:
      return "dig_2";
    case ControlState::finish_dig_2:
      return "finish_dig_2";
    case ControlState::navigate_to_hopper_2:
      return "navigate_to_hopper_2";
    case ControlState::dump_2:
      return "dump_2";
    case ControlState::finish:
      return "finish";
  }
}