#include <dig_control_2/dig_controller/dig_controller.h>
#include <dig_control_2/dig_params.h> // Can only include once

using std::clamp;
using std::abs;
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
      switch(dig_state)
      {
        case DigState::error:
        {
          ROS_ERROR("[dig][error] In error state, stopping all motion");
          stop();
          break;
        }
        case DigState::stowed:
        {
          // Neither of these systems should be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);
          if (abs(getBucketDuty()) > 0.0f) setBucketDuty(0.0f);

          switch (central_drive_state)
          {
            default:
              // TODO figure out what we do from here and where it is
              ROS_ERROR("[dig][stowed] This state is currently undefined");
              stop();
              break;
          }
          break;
        }
        case DigState::dig_transition:
        {
          // Neither of these systems should be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);
          if (abs(getBucketDuty()) > 0.0f) setBucketDuty(0.0f);

          // Only valid state here is if the backhoe is open
          if (backhoe_state != BackhoeState::open)
          {
            dig_state = DigState::error;
            stop();
            ROS_ERROR("[dig][dig_transition] Backhoe must be open while traveling downward to dig");
            break;
          }

          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            {
              ROS_ERROR("[dig][dig_transition] Near or at bottom limit switch during dig transition");
              // Shouldn't ever be near the bottom limit switches during this transition
              dig_state = DigState::error;
              stop();
              break;
            }
            case CentralDriveState::digging:
            {
              ROS_DEBUG("[dig][dig_transition] Start digging");
              dig_state = DigState::digging;
              stop();
              break;
            }
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::normal:
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              ROS_DEBUG("[dig][dig_transition] Continue moving down");
              setCentralDriveDuty(CentralDriveDuty::normal);
              setBackhoeDuty(0.0f);
              setBucketDuty(0.0f);
              setVibratorDuty(0.0f);
              break;
            }
          }
        }
        case DigState::digging:
        {
          // Neither of these systems should be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);
          if (abs(getBucketDuty()) > 0.0f) setBucketDuty(0.0f);

          if (backhoe_state != BackhoeState::open)
          {
            ROS_ERROR("[dig][digging] Backhoe must be open while digging");
            dig_state = DigState::error;
            stop();
            break;
          }

          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            {
              ROS_DEBUG("[dig][digging][near_bottom_limit] Moving to closing_backhoe state");
              dig_state = DigState::closing_backhoe;
              stop();
              break;
            }
            case CentralDriveState::digging:
            {
              // TODO implement this functionality
              // Dig until stalled for X seconds
              break;
            }
            case CentralDriveState::normal:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              ROS_ERROR("[dig][digging][default] Should not be in this central drive state");
              dig_state = DigState::error;
              stop();
              break;
            }
          }
          break;
        }
        case DigState::closing_backhoe:
        {
          /*
           * TODO this is where we stopped on Thursday
           */

          // Neither of these systems should be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);
          if (abs(getBucketDuty()) > 0.0f) setBucketDuty(0.0f);

          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            {
              // Transition to heading_to_dump when closed or stalled
              break;
            }
            case CentralDriveState::normal:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              // Something is wrong, error state
              break;
            }
          }
          break;
        }
        case DigState::dump_transition:
        {
          switch (central_drive_state)
          {
            case CentralDriveState::normal:
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            {
              // Make sure backhoe is closed, move upward
              break;
            }
            case CentralDriveState::near_dump_point:
            {
              // Make sure backhoe is closed, move upward slowly
              break;
            }
            case CentralDriveState::at_dump_point:
            {
              // Stop and enter dump state
              break;
            }
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              // Probably just destroyed the flaps, stop now
              break;
            }
          }
          break;
        }
        case DigState::dumping:
        {
          switch (central_drive_state)
          {
            case CentralDriveState::at_dump_point:
            {
              // Transition to moving_flaps_up when backhoe is open
              break;
            }
            case CentralDriveState::normal:
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              // Something bad happened, error state
              break;
            }
          }
          break;
        }
        case DigState::moving_flaps_up:
        {
          switch (central_drive_state)
          {
            case CentralDriveState::at_dump_point:
            {
              // Move up if backhoe is open
              break;
            }
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              // Transition either to stowed or into another dig
              break;
            }
            case CentralDriveState::normal:
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            case CentralDriveState::near_dump_point:
            {
              // Shouldn't be in any of these zones, error state
              break;
            }
          }
        }
      }




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
