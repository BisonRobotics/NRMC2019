#include <waypoint_control/waypoint_controller.h>

using namespace waypoint_control;

WaypointController::WaypointController(iVescAccess *front_left, iVescAccess *front_right,
                                       iVescAccess *back_right, iVescAccess *back_left)
{

}

WaypointController::~WaypointController()
{

}

void WaypointController::update()
{

}

void WaypointController::setControlState(ControlState state)
{

}

void WaypointController::stop()
{

}

ControlState WaypointController::getControlState() const
{
  return state;
}
