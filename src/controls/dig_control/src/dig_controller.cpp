#include <dig_control/dig_controller/dig_controller.h>
#include <dig_control/dig_params.h> // Can only include once
#include <boost/algorithm/clamp.hpp>

using boost::algorithm::clamp;
using std::abs;
using namespace dig_control;


DigController::DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                             iVescAccess *bucket_actuator, iVescAccess *vibrator)
{
  this->central_drive = central_drive;
  this->backhoe = backhoe_actuator;
  this->bucket = bucket_actuator;
  this->vibrator = vibrator;

  internally_allocated = false;

  bucket_state = BucketState::down;
  setControlState(ControlState::ready);
  update();
  if (goal_state != ControlState::ready)
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

void DigController::setControlState(DigController::ControlState goal)
{
  if (goal == ControlState::ready)
  {
    stop();
  }
  this->goal_state = goal;
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
  if (abs(bucket_duty) > 0.001f)
  {
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
  }
  // If duty is zero, assume that the bucket remains in same position
  // TODO check for stuck state
}

void DigController::update()
{
  // Update states
  updateCentralDriveState();
  updateBackhoeState();
  updateBucketState();

  // Handle state and goal
  switch (goal_state)
  {
    case ControlState::ready:
    {
      ROS_DEBUG("[ready] Ready for next goal");
      stop();
      break;
    }
    case ControlState::error:
    {
      ROS_ERROR("[error] Encountered error");
      stop();
      break;
    }
    case ControlState::manual:
    {
      ROS_DEBUG("[manual] Manually running dig controller");
      // Keep doing whatever is currently set,
      break;
    }
    case ControlState::dig:
    case ControlState::finish_dig:
    {
      // This system should never be on while digging
      if (abs(getBucketDuty()) > 0.0f) setBucketDuty(0.0f);

      switch(dig_state)
      {
        case DigState::stow:
        {
          // TODO this should probably actively stow the backhoe

          // This system should not be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);

          switch (central_drive_state)
          {
            case CentralDriveState::normal:
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            {
              ROS_ERROR("[dig][stowed][normal] Must be stowed with flaps up");
              goal_state = ControlState::error;
              stop();
              break;
            }
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              ROS_DEBUG("[dig][stowed][flaps_up] Stowed");
              goal_state = ControlState::ready;
              stop();
              break;
            }
          }
          break;
        }
        case DigState::dig_transition:
        {
          // This system should not be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);

          // Only valid state here is if the backhoe is open
          if (backhoe_state != BackhoeState::open)
          {
            goal_state = ControlState::error;
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
              goal_state = ControlState::error;
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
          break;
        }
        case DigState::digging:
        {
          // This system should not be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);

          if (backhoe_state != BackhoeState::open)
          {
            ROS_ERROR("[dig][digging] Backhoe must be open while digging");
            goal_state = ControlState::error;
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
              goal_state = ControlState::error;
              stop();
              break;
            }
          }
          break;
        }
        case DigState::closing_backhoe:
        {
          // Neither of these systems should be on in this state
          if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);

          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            {
              switch (backhoe_state)
              {
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][closing_backhoe][digging][stuck] Moving to dump transition");
                  dig_state = DigState::dump_transition;
                  stop();
                  break;
                }
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][closing_backhoe][digging][closed] Moving to dump_transition");
                  dig_state = DigState::dump_transition;
                  stop();
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                {
                  ROS_DEBUG("[dig][closing_backhoe][digging][open] Closing backhoe");
                  setBackhoeDuty(BackhoeDuty::normal);
                  break;
                }
              }
              break;
            }
            case CentralDriveState::normal:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              ROS_ERROR("[dig][closing_backhoe][default] Should not be in this central drive state");
              goal_state = ControlState::error;
              stop();
              break;
            }
          }
          break;
        }
        case DigState::dump_transition:
        {
          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            {
              if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);

              switch (backhoe_state)
              {
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][dump_transition][digging][closed] Moving to dump_transition");
                  setBackhoeDuty(0.0f);
                  setCentralDriveDuty(-CentralDriveDuty::normal);
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][digging][open] "
                           "Trying to close stuck backhoe while transitioning to dump");
                  setBackhoeDuty(BackhoeDuty::normal);
                  setCentralDriveDuty(-CentralDriveDuty::normal);
                  break;
                }
              }
              break;
            }
            case CentralDriveState::normal:
            {
              if (abs(getVibratorDuty()) > 0.0f) setVibratorDuty(0.0f);

              // Only keep moving if backhoe is closed
              switch (backhoe_state)
              {
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][dump_transition][digging][closed] Moving to dump_transition");
                  setBackhoeDuty(0.0f);
                  setCentralDriveDuty(-CentralDriveDuty::normal);
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                case BackhoeState::stuck:
                {
                  // TODO should probably have behavior to timeout and attempt something else
                  ROS_WARN("[dig][dump_transition][normal][open] Trying to close backhoe before continuing");
                  setBackhoeDuty(BackhoeDuty::normal);
                  setCentralDriveDuty(0.0f);
                  break;
                }
              }
              break;
            }
            case CentralDriveState::near_dump_point:
            {
              // Get vibrator started
              setVibratorDuty(VibratorDuty::normal);

              switch (backhoe_state)
              {
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][dump_transition][near_dump_point][closed] Moving to dump slowly");
                  setBackhoeDuty(0.0f);
                  setCentralDriveDuty(-CentralDriveDuty::slow);
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][near_dump_point][open] "
                           "Can't keep moving upward with the backhoe open");
                  goal_state = ControlState::error;
                  stop();
                  break;
                }
              }
              break;
            }
            case CentralDriveState::at_dump_point:
            {
              // Run vibrator
              setVibratorDuty(VibratorDuty::normal);

              switch (backhoe_state)
              {
                case BackhoeState::closed:
                case BackhoeState::traveling:
                {
                  ROS_DEBUG("[dig][dump_transition][at_dump_point][closed] Opening backhoe");
                  setCentralDriveDuty(0.0f);
                  setBackhoeDuty(-BackhoeDuty::normal);
                  break;
                }
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][at_dump_point][stuck] Trying to open backhoe");
                  setCentralDriveDuty(0.0f);
                  setBackhoeDuty(-BackhoeDuty::normal);
                  break;
                }
                case BackhoeState::open:
                {
                  ROS_DEBUG("[dig][dump_transition][at_dump_point][open] Transitioning to moving_flaps_up");
                  dig_state = DigState::moving_flaps_up;
                  stop();
                  break;
                }
              }
              break;
            }
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              ROS_ERROR("[dig][dump_transition][flaps_up] Should not be in this state");
              goal_state = ControlState::error;
              stop();
              break;
            }
          }
          break;
        }
        case DigState::moving_flaps_up:
        {
          // Run vibrator
          setVibratorDuty(VibratorDuty::normal);

          // Make sure backhoe is open
          if (backhoe_state != BackhoeState::open)
          {
            ROS_ERROR("[dig][moving_flaps_up] Backhoe should be open at this point");
            goal_state = ControlState::error;
            stop();
            break;
          }

          switch (central_drive_state)
          {
            case CentralDriveState::at_dump_point:
            {
              ROS_DEBUG("[dig][moving_flaps_up][at_dump_point] Moving flaps up");
              setCentralDriveDuty(-CentralDriveDuty::normal);
              setBackhoeDuty(0.0f);
              break;
            }
            case CentralDriveState::flaps_up:
            case CentralDriveState::near_top_limit:
            case CentralDriveState::at_top_limit:
            {
              stop();
              if (goal_state == ControlState::dig)
              {
                ROS_DEBUG("[dig][moving_flaps_up][flaps_up] Starting another dig");
                dig_state = DigState::dig_transition;
              }
              else if (goal_state == ControlState::finish_dig)
              {
                ROS_DEBUG("[dig][moving_flaps_up][flaps_up] Dig finished");
                dig_state = DigState::stow;
              }
              break;
            }
            case CentralDriveState::normal:
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::near_bottom_limit:
            case CentralDriveState::digging:
            case CentralDriveState::near_dump_point:
            {
              ROS_ERROR("[dig][moving_flaps_up][near_dump_point] Should not be in this state");
              stop();
              goal_state = ControlState::error;
              break;
            }
          }
          break;
        }
      }
      break;
    }
    case ControlState::dump:
    {
      setCentralDriveDuty(0.0f);
      setBackhoeDuty(0.0f);
      setVibratorDuty(VibratorDuty::normal);
      switch (bucket_state)
      {
        case BucketState::up:
        {
          ROS_DEBUG("[dump][up] Transitioning to finish dump");
          goal_state = ControlState::finish_dump;
          stop();
          break;
        }
        case BucketState::down:
        case BucketState::traveling:
        {
          ROS_DEBUG("[dump][traveling] Moving bucket up");
          setBucketDuty(BucketDuty::normal);
          break;
        }
        case BucketState::stuck:
        {
          ROS_ERROR("[dump][stuck] Bucket is stuck, keep trying");
          setBucketDuty(BucketDuty::normal);
          break;
        }
      }
      break;
    }
    case ControlState::finish_dump:
    {
      setCentralDriveDuty(0.0f);
      setBackhoeDuty(0.0f);
      setVibratorDuty(VibratorDuty::normal);
      switch (bucket_state)
      {
        case BucketState::up:
        case BucketState::traveling:
        {
          ROS_DEBUG("[dump][traveling] Finishing dump");
          setBucketDuty(-BucketDuty::normal);
          break;
        }
        case BucketState::down:
        {
          ROS_DEBUG("[dump][down] Finished dump");
          goal_state = ControlState::ready;
          stop();
          break;
        }
        case BucketState::stuck:
        {
          ROS_ERROR("[dump][stuck] Bucket is stuck, keep trying");
          setBucketDuty(-BucketDuty::normal);
          break;
        }
      }
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

DigController::ControlState DigController::getControlState() const
{
  return goal_state;
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
  central_drive->setCustom(central_drive_duty);
}

void DigController::setBackhoeDuty(float value)
{
  backhoe_duty = clamp(value, -MAX_BACKHOE_DUTY, MAX_BACKHOE_DUTY);
  backhoe->setCustom(backhoe_duty);
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
