#include <dig_control_2/dig_controller/dig_controller.h>
#include <dig_control_2/dig_params.h> // Can only include once


using namespace dig_control_2;


DigController::DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                             iVescAccess *bucket_actuator, iVescAccess *vibrator)
{
  this->central_drive = central_drive;
  this->backhoe_actuator = backhoe_actuator;
  this->bucket_actuator = bucket_actuator;
  this->vibrator = vibrator;

  internally_allocated = false;
}

DigController::DigController() :
    DigController(new VescAccess(central_drive_param),   new VescAccess(backhoe_actuator_param),
                  new VescAccess(bucket_actuator_param), new VescAccess(vibrator_param))
{
  internally_allocated = true;
}

DigController::~DigController()
{
  if (internally_allocated)
  {
    // TODO should have virtual destructor defined, have to do a cast since we don't
    delete (VescAccess*)central_drive;
    delete (VescAccess*)backhoe_actuator;
    delete (VescAccess*)bucket_actuator;
    delete (VescAccess*)vibrator;
  }
}

bool DigController::isInternallyAllocated()
{
  return internally_allocated;
}