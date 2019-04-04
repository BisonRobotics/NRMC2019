#include <dig_control_2/dig_controller/dig_controller.h>
#include <dig_control_2/dig_params.h> // Can only include once

using std::clamp;
using namespace dig_control_2;


DigController::DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                             iVescAccess *bucket_actuator, iVescAccess *vibrator)
{
  this->central_drive = central_drive;
  this->backhoe = backhoe_actuator;
  this->bucket = bucket_actuator;
  this->vibrator = vibrator;

  internally_allocated = false;

  setGoal(Goal::stop);
  update();
  if (status != Status::ready)
  {
    throw std::runtime_error("Something went wrong updating the dig controller state");
  }
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
    delete (VescAccess*)backhoe;
    delete (VescAccess*)bucket;
    delete (VescAccess*)vibrator;
  }
}

bool DigController::isInternallyAllocated()
{
  return internally_allocated;
}

void DigController::setGoal(DigController::Goal goal)
{
  if (goal == Goal::stop)
  {
    stop();
  }
  this->goal = goal;
}

void DigController::updateCentralDriveState()
{
  nsVescAccess::limitSwitchState limit_switch_state = central_drive->getLimitSwitchState();
  bool top_limit = (limit_switch_state == nsVescAccess::limitSwitchState::topOfMotion);
  bool bottom_limit = (limit_switch_state == nsVescAccess::limitSwitchState::bottomOfMotion);
  float angle = central_drive->getPotPosition();

  if (top_limit || angle >= CentralDriveAngles::top_limit)
  {
    central_drive_state = CentralDriveState::at_top_limit;
  }
  else if (bottom_limit || angle <= CentralDriveAngles::bottom_limit)
  {
    central_drive_state = CentralDriveState::at_bottom_limit;
  }
  else if (angle > CentralDriveAngles::bottom_limit && angle <= CentralDriveAngles::digging_bottom)
  {
    central_drive_state = CentralDriveState::near_bottom_limit;
  }
  else if (angle > CentralDriveAngles::digging_bottom && angle <= CentralDriveAngles::digging_top)
  {
    central_drive_state = CentralDriveState::digging;
  }
  else if (angle > CentralDriveAngles::dump_bottom && angle < CentralDriveAngles::dump_point)
  {
    central_drive_state = CentralDriveState::near_dump_point;
  }
  else if (angle >= CentralDriveAngles::dump_point && angle <= CentralDriveAngles::dump_top)
  {
    central_drive_state = CentralDriveState::at_dump_point;
  }
  else if (angle > CentralDriveAngles::dump_top && angle <= CentralDriveAngles::near_top_limit)
  {
    central_drive_state = CentralDriveState::flaps_up;
  }
  else if (angle > CentralDriveAngles::near_top_limit && angle <= CentralDriveAngles::top_limit)
  {
    central_drive_state = CentralDriveState::near_top_limit;
  }
  else
  {
    central_drive_state = CentralDriveState::normal;
  }

  // TODO add stuck checking
}

void DigController::updateBackhoeState()
{
  nsVescAccess::limitSwitchState limit_switch_state = backhoe->getLimitSwitchState();
  bool top_limit = (limit_switch_state == nsVescAccess::limitSwitchState::topOfMotion);
  bool bottom_limit = (limit_switch_state == nsVescAccess::limitSwitchState::bottomOfMotion);
  double velocity = backhoe->getLinearVelocity();

  if (top_limit)
  {
    backhoe_state = BackhoeState::closed;
  }
  else if (bottom_limit)
  {
    backhoe_state = BackhoeState::open;
  }
  else if (std::abs(velocity) > 0.001) // TODO figure out a good number for this
  {
    backhoe_state = BackhoeState::traveling;
  }
  else
  {
    backhoe_state = BackhoeState::stuck;
  }
}

void DigController::updateBucketState()
{
  float torque = std::abs(bucket->getTorque());
  if (bucket_duty > 0.0f && torque < 0.001f)
  {
    bucket_state = BucketState::up;
  }
  else if (bucket_duty < 0.0f && torque < 0.001f)
  {
    bucket_state = BucketState::down;
  }
  else
  {
    bucket_state = BucketState::traveling;
  }
  // TODO check for stuck state
}

void DigController::update()
{
  // Update states
  updateCentralDriveState();
  updateBackhoeState();
  updateBucketState();

  // Handle state and goal
  switch (goal)
  {
    case Goal::stop:
    {
      stop();
      break;
    }
    case Goal::manual:
    {
      // Keep doing whatever is currently set,
      break;
    }
    case Goal::drive_pose:
    {
      // TODO figure out what position this should be
      break;
    }
    case Goal::dig:
    {
      switch (central_drive_state)
      {
        case CentralDriveState::near_dump_point:
        {
          switch (backhoe_state)
          {
            case BackhoeState::closed:
            {
              setCentralDriveDuty(-CentralDriveDuty::slow);
              setBackhoeDuty(0.0f);
              setBucketDuty(0.0f);
              setVibratorDuty(0.0f);
            }
            case BackhoeState::traveling:
            case BackhoeState::open:
            case BackhoeState::stuck:
            {
              stop();
              status = Status::error;
              ROS_ERROR("Backhoe not closed near dump point");
            }
          }
          break;
        }
        case CentralDriveState::at_dump_point:
        {
          setBucketDuty(0.0f);
          setVibratorDuty(VibratorDuty::normal);

          switch (backhoe_state)
          {
            case BackhoeState::closed:
            case BackhoeState::traveling:
            {
              // Open backhoe
              setCentralDriveDuty(0.0f);
              setBackhoeDuty(-BackhoeDuty::normal);
              ROS_DEBUG("Opening backhoe");
              break;
            }
            case BackhoeState::open:
            {
              // Open flaps
              central_drive_duty = -CentralDriveDuty::normal;
              backhoe_duty = 0.0f;
              ROS_DEBUG("Opening flaps");
              break;
            }
            case BackhoeState::stuck:
            {
              stop();
              status = Status::error;
              ROS_ERROR("Backhoe stuck while trying to open")
            }
          }
          break;
        }
        case CentralDriveState::at_top_limit:
        case CentralDriveState::near_top_limit:
        case CentralDriveState::normal:
        {

        }

      }
      break;
    }
    case Goal::dump:
    {

      break;
    }
  }
}

void DigController::stop()
{
  setCentralDriveDuty(0.0f);
  setBackhoeDuty(0.0f);
  setBucketDuty(0.0f);
  setVibratorDuty(0.0f);
}

DigController::Goal DigController::getGoal() const
{
  return goal;
}

DigController::Status DigController::getStatus() const
{
  return status;
}

double DigController::getProgress() const
{
  return progress;
}

DigController::CentralDriveState DigController::getCentralDriveState() const
{
  return central_drive_state;
}

DigController::BackhoeState DigController::getBackhoeState() const
{
  return backhoe_state;
}

DigController::BucketState DigController::getBucketState() const
{
  return bucket_state;
}

void DigController::setCentralDriveDuty(float value)
{
  central_drive_duty = clamp(value, -MAX_CENTRAL_DRIVE_DUTY, MAX_CENTRAL_DRIVE_DUTY);
  central_drive->setDuty(central_drive_duty);
}

void DigController::setBackhoeDuty(float value)
{
  backhoe_duty = clamp(value, -MAX_BACKHOE_DUTY, MAX_BACKHOE_DUTY);
  backhoe->setDuty(backhoe_duty);
}

void DigController::setBucketDuty(float value)
{
  bucket_duty = clamp(value, -MAX_BUCKET_DUTY, MAX_BUCKET_DUTY);
  bucket->setDuty(bucket_duty);
}

void DigController::setVibratorDuty(float value)
{
  vibrator_duty = clamp(value, -MAX_VIBRATOR_DUTY, MAX_VIBRATOR_DUTY);
  vibrator->setDuty(vibrator_duty);
}

float DigController::getCentralDriveDuty() const
{
  return central_drive_duty;
}

float DigController::getBackhoeDuty() const
{
  return backhoe_duty;
}

float DigController::getBucketDuty() const
{
  return bucket_duty;
}

float DigController::getVibratorDuty() const
{
  return vibrator_duty;
}
