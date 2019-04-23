#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>

#include <utilities/filter.h>
#include <waypoint_control/waypoint_control_states.h>

namespace waypoint_control
{
  class WaypointController
  {
  public:
    WaypointController(iVescAccess *front_left,   iVescAccess *front_right,
                  iVescAccess *back_right, iVescAccess *back_left);
    ~WaypointController();

    void update();

    void setControlState(ControlState goal) ;
    void stop();
    ControlState getControlState() const ;

  private:
    iVescAccess *front_left, *front_right, *back_right, *back_left;
    ControlState state;
  };

}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROLLER_H
