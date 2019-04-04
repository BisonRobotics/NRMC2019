#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>

namespace dig_control_2
{
  class DigController
  {
  public:

    DigController();
    DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator);
    ~DigController();

    bool isInternallyAllocated();

  private:
    iVescAccess *central_drive, *backhoe_actuator;
    iVescAccess *bucket_actuator, *vibrator;
    bool internally_allocated;
  };

}

#endif //DIG_CONTROL_2_DIG_CONTROLLER_H
