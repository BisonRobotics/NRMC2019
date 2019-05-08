#include <dig_control/dig_controller.h>
#include <dig_control/dig_params.h> // Can only include once
#include <boost/algorithm/clamp.hpp>
#include <utilities/utilities.h>
#include <utilities/filter.h>


using boost::algorithm::clamp;
using std::abs;
using namespace dig_control;
using utilities::simpleLowPassFilter;

// TODO add initialize mode

DigController::DigController(Config config, iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                             iVescAccess *bucket_actuator, iVescAccess *vibrator) : 
  config(config), battery_voltage(0.0)
{
  this->central_drive = central_drive;
  this->backhoe = backhoe_actuator;
  this->bucket = bucket_actuator;
  this->vibrator = vibrator;

  ROS_INFO("[DigController::DigController]: Waiting for VESCs to come online");
  while (ros::ok())
  {
    if (central_drive->isAlive() && backhoe->isAlive() && bucket->isAlive() && vibrator->isAlive())
    {
      break;
    }
    ROS_WARN("[DigController::DigController]: VESCs not online");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("[DigController::DigController]: VESCs online");

  central_current = 0.0f;
  backhoe_current = 0.0f;
  bucket_current = 0.0f;
  vibrator_current = 0.0f;
  bucket_position = 10.0f;
  last_bucket_state_change = ros::Time::now();

  internally_allocated = false;

  backhoe_stuck_count = 0;
  bucket_state = BucketState::down;
  dig_state = DigState::dig_transition;
  setControlState(ControlState::ready);
  update();
  if (goal_state != ControlState::ready)
  {
    throw std::runtime_error("Something went wrong updating the dig controller state");
  }
}

DigController::DigController(Config config) :
    DigController(config, new VescAccess(central_drive_param),   new VescAccess(backhoe_actuator_param),
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

void DigController::setControlState(ControlState goal)
{
  if (goal == ControlState::ready)
  {
    stop();
  }
  else if (goal == ControlState::dig)
  {
    dig_state = DigState::initialize;
  }
  this->goal_state = goal;
}

void DigController::updateCentralDriveState()
{
  nsVescAccess::limitSwitchState limit_switch_state = central_drive->getLimitSwitchState();
  bool top_limit = (limit_switch_state == nsVescAccess::limitSwitchState::topOfMotion);
  bool bottom_limit = (limit_switch_state == nsVescAccess::limitSwitchState::bottomOfMotion);
  central_drive_position = central_drive->getADC();
  double current = central_drive->getCurrent();
  if (std::abs(current) < 100.0f)
  {
    simpleLowPassFilter<double>(central_current, current, config.centralDriveCurrentFilterK());
  }

  if (top_limit || central_drive_position >= config.centralDriveAngles().top_limit)
  {
    central_drive_state = CentralDriveState::at_top_limit;
  }
  else if (bottom_limit || central_drive_position <= config.centralDriveAngles().bottom_limit ||
          (config.floorTest()   && central_drive_position <= config.centralDriveAngles().floor_limit))
  {
    central_drive_state = CentralDriveState::at_bottom_limit;
  }
  else if (central_drive_position >= config.centralDriveAngles().digging_bottom + config.centralDriveAngles().variation &&
           central_drive_position <  config.centralDriveAngles().digging_top    - config.centralDriveAngles().variation)
  {
    central_drive_state = CentralDriveState::digging;
  }
  else if (central_drive_position >= config.centralDriveAngles().digging_top + config.centralDriveAngles().variation &&
           central_drive_position < config.centralDriveAngles().flaps_bottom - config.centralDriveAngles().variation)
  {
    central_drive_state = CentralDriveState::near_digging;
  }
  else if (central_drive_position >= config.centralDriveAngles().flaps_bottom + config.centralDriveAngles().variation &&
           central_drive_position < config.centralDriveAngles().dump_bottom   - config.centralDriveAngles().variation)
  {
    central_drive_state = CentralDriveState::flap_transition_down;
  }
  else if (central_drive_position >= config.centralDriveAngles().dump_bottom + config.centralDriveAngles().variation &&
           central_drive_position < config.centralDriveAngles().dump_point   - config.centralDriveAngles().variation)
  {
    central_drive_state = CentralDriveState::near_dump_point;
  }
  else if (central_drive_position >= config.centralDriveAngles().dump_point + config.centralDriveAngles().variation &&
           central_drive_position < config.centralDriveAngles().dump_top    - config.centralDriveAngles().variation)
  {
    central_drive_state = CentralDriveState::at_dump_point;
  }
  else if (central_drive_position >= config.centralDriveAngles().dump_top  + config.centralDriveAngles().variation &&
           central_drive_position <= config.centralDriveAngles().top_limit - config.centralDriveAngles().variation)
  {
    central_drive_state = CentralDriveState::flap_transition_up;
  }
  // TODO add stuck checking
}

void DigController::updateBackhoeState()
{
  nsVescAccess::limitSwitchState limit_switch_state = backhoe->getLimitSwitchState();
  bool top_limit = (limit_switch_state == nsVescAccess::limitSwitchState::topOfMotion);
  bool bottom_limit = (limit_switch_state == nsVescAccess::limitSwitchState::bottomOfMotion);
  double velocity = backhoe->getLinearVelocity();
  double current = backhoe->getCurrent();
  if (std::abs(current) < 100.0f)
  {
    simpleLowPassFilter<double>(backhoe_current, current, config.currentFilterK());
  }

  if (top_limit)
  {
    backhoe_state = BackhoeState::closed;
    backhoe_stuck_count = 0;
  }
  else if (bottom_limit)
  {
    backhoe_state = BackhoeState::open;
    backhoe_stuck_count = 0;
  }
  else
  {
    backhoe_state = BackhoeState::traveling;
  }
}

void DigController::updateBucketState()
{
  double current = bucket->getCurrent();
  if (std::abs(current) < 100.0f)
  {
    simpleLowPassFilter<double>(bucket_current, current, config.bucketFilterK());
  }
  bool debounce = ros::Time::now() - last_bucket_state_change >= ros::Duration(1.0f);
  if (abs(bucket_duty) > 0.001f)
  {
    if (std::abs(bucket_current) >= 0.4)
    {
      if (bucket_state != BucketState::traveling && debounce)
      {
        last_bucket_state_change = ros::Time::now();
        bucket_state = BucketState::traveling;
      }
      // 14 seconds to travel from 10 inches to 16 inches
      double progress = (double)((ros::Time::now() - last_bucket_state_change).toSec() / 14.0 * 6.0);
      bucket_position = (bucket_duty >= 0.0f) ? progress + 10.0f : 16.0f - progress;
    }
    else if (bucket_duty > 0.0f && bucket_state != BucketState::down)
    {
      if (bucket_state != BucketState::up && debounce)
      {
        last_bucket_state_change = ros::Time::now();
        bucket_state = BucketState::up;
      }
      bucket_position = 16.0f;
    }
    else if (bucket_duty < 0.0f && bucket_state != BucketState::up)
    {
      if (bucket_state != BucketState::down && debounce)
      {
        last_bucket_state_change = ros::Time::now();
        bucket_state = BucketState::down;
      }
      bucket_position = 10.0f;
    }
  }
  // If duty is zero, assume that the bucket remains in same position
  // TODO check for stuck state
}


void DigController::updateVibratorState()
{
  double current = vibrator->getCurrent();
  if (std::abs(current) < 100.0f)
  {
    simpleLowPassFilter<double>(vibrator_current, current, config.currentFilterK());
  }
}

void DigController::updateBatteryVoltage()
{
  double sample = (central_drive->getVin() + backhoe->getVin() + bucket->getVin() + vibrator->getVin()) / 4.0;
  simpleLowPassFilter<double>(battery_voltage, sample, config.batteryFilterK());
  if (battery_voltage < config.minVoltage())
  {
    ROS_WARN("[DigController::updateBatteryVoltage]: Vin = %f < %f", battery_voltage, config.minVoltage());
    battery_voltage = config.startVoltage();
  }
}

void DigController::update()
{
  // Update states
  updateCentralDriveState();
  updateBackhoeState();
  updateBucketState();
  updateVibratorState();
  updateBatteryVoltage();

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
      switch (central_drive_state)
      {
        case CentralDriveState::at_bottom_limit:
        {
          if (central_drive_duty < 0.0f)
          {
            setCentralDriveDuty(0.0f);
          }
          break;
        }
        case CentralDriveState::at_top_limit:
        {
          if (central_drive_duty > 0.0f)
          {
            setCentralDriveDuty(0.0);
          }
          break;
        }
        case CentralDriveState::digging:
        case CentralDriveState::near_digging:
        case CentralDriveState::flap_transition_down:
        case CentralDriveState::near_dump_point:
        case CentralDriveState::at_dump_point:
        case CentralDriveState::flap_transition_up:
        {
          // Do nothing
          break;
        }

      }
      // Keep doing whatever is currently set,
      break;
    }
    case ControlState::dig:
    case ControlState::finish_dig:
    {
      // This system should never be on while digging
      if (abs(getBucketDuty()) > 0.0f) setBucketDuty(0.0f);

      // Keep on entire time
      setVibratorDuty(config.vibratorDuty().normal);

      switch(dig_state)
      {
        case DigState::stow:
        {
          switch (central_drive_state)
          {

            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::digging:
            {
              ROS_ERROR("[dig][stowed][digging] Should not be attempting to stow backhoe in digging position");
              //goal_state = ControlState::error;
              //stop();
              break;
            }
            case CentralDriveState::near_digging:
            case CentralDriveState::flap_transition_down:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flap_transition_up:
            case CentralDriveState::at_top_limit:
            {
              if (central_drive_position <= config.centralDriveAngles().stow_position)
              {
                setCentralDriveDuty(0.0f);
                if (getBackhoePosition() > 100)
                {
                  goal_state = ControlState::ready;
                  dig_state = DigState::dig_transition;

                }
                else
                {
                  setBackhoeDuty(config.backhoeDuty().slow);
                }
              }
              else
              {
                setCentralDriveDuty(-config.centralDriveDuty().normal);
              }
              break;
            }

          }
          break;
        }
        case DigState::initialize:
        {
          if (backhoe_state != BackhoeState::open)
          {
            setBackhoeDuty(-config.backhoeDuty().normal);
          }
          else
          {
            dig_state = DigState::dig_transition;
          }
          break;
        }
        case DigState::dig_transition:
        {

          // Only valid state here is if the backhoe is open
          if (backhoe_state != BackhoeState::open)
          {
            ROS_ERROR("[dig][dig_transition] Backhoe must be open while traveling downward to dig");
            //goal_state = ControlState::error;
            //stop();
            break;
          }

          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::digging:
            {
              ROS_DEBUG("[dig][dig_transition] Start digging");
              dig_state = DigState::digging;
              //stop();
              break;
            }
            case CentralDriveState::near_digging:
            case CentralDriveState::flap_transition_down:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flap_transition_up:
            case CentralDriveState::at_top_limit:
            {
              ROS_DEBUG("[dig][dig_transition] Continue moving down");
              setCentralDriveDuty(-config.centralDriveDuty().fast);
              setBackhoeDuty(0.0f);
              setBucketDuty(0.0f);
              break;
            }
          }
          break;
        }
        case DigState::digging:
        {
          if (backhoe_state != BackhoeState::open)
          {
            ROS_ERROR("[dig][digging] Backhoe must be open while digging");
            //goal_state = ControlState::error;
            //stop();
            break;
          }

          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            {
              ROS_DEBUG("[dig][digging][near_bottom_limit] Moving to closing_backhoe state");
              setCentralDriveDuty(0.0);
              dig_state = DigState::closing_backhoe;
              break;
            }
            case CentralDriveState::digging:
            {
              setCentralDriveDuty(-config.centralDriveDuty().fast);
              if (central_current > config.centralDriveDigCurrentThreshold())
              {
                dig_state = DigState::closing_backhoe;
              }
              break;
            }
            case CentralDriveState::near_digging:
            {
              ROS_WARN("[dig][digging][near_digging] Should not be in this state");
              break;
            }
            case CentralDriveState::flap_transition_down:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flap_transition_up:
            case CentralDriveState::at_top_limit:
            {
              ROS_ERROR("[dig][digging][default] Should not be in this state");
              //goal_state = ControlState::error;
              //stop();
              break;
            }
          }
          break;
        }
        case DigState::closing_backhoe:
        {
          switch (central_drive_state)
          {
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::digging:
            case CentralDriveState::near_digging:
            {
              setCentralDriveDuty(config.centralDriveDuty().ultra_slow);
              switch (backhoe_state)
              {
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][closing_backhoe][digging][stuck] Moving to dump transition");
                  dig_state = DigState::dump_transition;
                  break;
                }
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][closing_backhoe][digging][closed] Moving to dump_transition");
                  dig_state = DigState::dump_transition;
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                {
                  ROS_DEBUG("[dig][closing_backhoe][digging][open] Closing backhoe");
                  setBackhoeDuty(config.backhoeDuty().normal);
                  break;
                }
              }
              break;
            }
            case CentralDriveState::flap_transition_down:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flap_transition_up:
            case CentralDriveState::at_top_limit:
            {
              ROS_ERROR("[dig][closing_backhoe][default] Should not be in this central drive state");
              //goal_state = ControlState::error;
              //stop();
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
            case CentralDriveState::digging:
            {
              switch (backhoe_state)
              {
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][dump_transition][digging][closed] Moving to dump_transition");
                  setBackhoeDuty(config.backhoeDuty().normal);
                  setCentralDriveDuty(config.centralDriveDuty().fast);
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][digging][open] "
                           "Trying to close stuck backhoe while transitioning to dump");
                  setBackhoeDuty(config.backhoeDuty().normal);
                  setCentralDriveDuty(config.centralDriveDuty().normal);
                  break;
                }
              }
              break;
            }
            case CentralDriveState::near_digging:
            {
              // Only keep moving if backhoe is closed
              switch (backhoe_state)
              {
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][dump_transition][digging][closed] Moving to dump_transition");
                  setBackhoeDuty(config.backhoeDuty().normal);
                  setCentralDriveDuty(config.centralDriveDuty().fast);
                  break;
                }
                case BackhoeState::stuck:
                case BackhoeState::traveling:
                case BackhoeState::open:
                {
                  ROS_WARN("[dig][dump_transition][normal][open] Trying to close backhoe while continuing");
                  setBackhoeDuty(config.backhoeDuty().fast); // TODO check current during this maneuver?
                  setCentralDriveDuty(config.centralDriveDuty().normal);
                  break;
                }
              }
              break;
            }
            case CentralDriveState::flap_transition_down: // TODO can move faster for this portion
            {

              switch (backhoe_state)
              {
                case BackhoeState::closed:
                {
                  ROS_DEBUG("[dig][dump_transition][near_dump_point][closed] Moving to dump slowly");
                  setBackhoeDuty(config.backhoeDuty().normal);
                  setCentralDriveDuty(config.centralDriveDuty().slowish);
                  break;
                }
                case BackhoeState::traveling:
                case BackhoeState::open:
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][near_dump_point][open] "
                           "Can't keep moving upward with the backhoe open");
                  //goal_state = ControlState::error;
                  //stop();
                  break;
                }
              }
              break;
            }
            case CentralDriveState::near_dump_point:
            {

              switch (backhoe_state)
              {
                case BackhoeState::closed:
                case BackhoeState::traveling:
                {
                  ROS_DEBUG("[dig][dump_transition][near_dump_point][closed] Opening backhoe");
                  setCentralDriveDuty(config.centralDriveDuty().slowish);
                  setBackhoeDuty(-config.backhoeDuty().normal);
                  break;
                }
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][near_dump_point][stuck] Trying to open backhoe");
                  setCentralDriveDuty(config.centralDriveDuty().slowish);
                  setBackhoeDuty(-config.backhoeDuty().normal);
                  break;
                }
                case BackhoeState::open:
                {
                  ROS_DEBUG("[dig][dump_transition][near_dump_point][open] Transitioning to moving_flaps_up");
                  dig_state = DigState::moving_flaps_up;
                  stop();
                  break;
                }
              }
              break;
            }
            case CentralDriveState::at_dump_point:
            {

              switch (backhoe_state)
              {
                case BackhoeState::closed:
                case BackhoeState::traveling:
                {
                  ROS_DEBUG("[dig][dump_transition][at_dump_point][closed] Opening backhoe");
                  setCentralDriveDuty(0.0f);
                  setBackhoeDuty(-config.backhoeDuty().normal);
                  break;
                }
                case BackhoeState::stuck:
                {
                  ROS_WARN("[dig][dump_transition][at_dump_point][stuck] Trying to open backhoe");
                  setCentralDriveDuty(0.0f);
                  setBackhoeDuty(-config.backhoeDuty().normal);
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
            case CentralDriveState::flap_transition_up:
            case CentralDriveState::at_top_limit:
            {
              ROS_ERROR("[dig][dump_transition][flaps_up] Should not be in this state");
              //goal_state = ControlState::error;
              //stop();
              break;
            }
          }
          break;
        }
        case DigState::moving_flaps_up:
        {

          // Make sure backhoe is open
          if (backhoe_state != BackhoeState::open)
          {
            ROS_ERROR("[dig][moving_flaps_up] Backhoe should be open at this point");
            //goal_state = ControlState::error;
            //stop();
            break;
          }

          switch (central_drive_state)
          {
            case CentralDriveState::near_digging:
            case CentralDriveState::flap_transition_down:
            case CentralDriveState::near_dump_point:
            case CentralDriveState::at_dump_point:
            case CentralDriveState::flap_transition_up:
            {
              ROS_DEBUG("[dig][moving_flaps_up][at_dump_point] Moving flaps up");
              setCentralDriveDuty(config.centralDriveDuty().normal);
              setBackhoeDuty(0.0f);
              break;
            }
            case CentralDriveState::at_top_limit:
            {
              setCentralDriveDuty(0.0f);
              setBackhoeDuty(0.0f);
              ros::Duration(3.0).sleep();
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
            case CentralDriveState::at_bottom_limit:
            case CentralDriveState::digging:
            {
              ROS_ERROR("[dig][moving_flaps_up][near_dump_point] Should not be in this state");
              //stop();
              //goal_state = ControlState::error;
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
      setVibratorDuty(config.vibratorDuty().normal);
      switch (bucket_state)
      {
        case BucketState::up:
        {
          ROS_DEBUG("[dump][up] Transitioning to finish dump");
          goal_state = ControlState::finish_dump;
          stop();
          ros::Duration(4.0).sleep();
          break;
        }
        case BucketState::down:
        case BucketState::traveling:
        {
          ROS_DEBUG("[dump][traveling] Moving bucket up");
          setBucketDuty(config.bucketDuty().fast);
          break;
        }
        case BucketState::stuck:
        {
          ROS_ERROR("[dump][stuck] Bucket is stuck, keep trying");
          setBucketDuty(config.bucketDuty().fast);
          break;
        }
      }
      break;
    }
    case ControlState::finish_dump:
    {
      setCentralDriveDuty(0.0f);
      setBackhoeDuty(0.0f);
      setVibratorDuty(0.0f);
      switch (bucket_state)
      {
        case BucketState::up:
        case BucketState::traveling:
        {
          ROS_DEBUG("[dump][traveling] Finishing dump");
          setBucketDuty(-config.bucketDuty().fast);
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
          setBucketDuty(-config.bucketDuty().fast);
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

ControlState DigController::getControlState() const
{
  return goal_state;
}

CentralDriveState DigController::getCentralDriveState() const
{
  return central_drive_state;
}

BackhoeState DigController::getBackhoeState() const
{
  return backhoe_state;
}

BucketState DigController::getBucketState() const
{
  return bucket_state;
}

DigState DigController::getDigState() const
{
  return dig_state;
}


double DigController::voltageCompensation(double duty)
{
  if (config.voltageCompensation())
  {
    return clamp(duty * config.startVoltage() / battery_voltage,
                 -config.maxCompensatedDuty(), config.maxCompensatedDuty());
  }
  else
  {
    return duty;
  }
}

void DigController::setCentralDriveDuty(double value)
{
  switch (central_drive_state)
  {
    case CentralDriveState::at_bottom_limit:
    {
      central_drive_duty = clamp(value, 0.0f, config.centralDriveDuty().max);
      break;
    }
    case CentralDriveState::at_top_limit:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, 0.0f);
      break;
    }
    case CentralDriveState::digging:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, config.centralDriveDuty().max);
      break;
    }
    case CentralDriveState::near_digging:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, config.centralDriveDuty().max);
      break;
    }
    case CentralDriveState::flap_transition_down:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, config.centralDriveDuty().slowish);
      break;
    }
    case CentralDriveState::near_dump_point:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, config.centralDriveDuty().slowish);
      break;
    }
    case CentralDriveState::at_dump_point:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, config.centralDriveDuty().slow);
      break;
    }
    case CentralDriveState::flap_transition_up:
    {
      central_drive_duty = clamp(value, -config.centralDriveDuty().max, config.centralDriveDuty().slowish);
      break;
    }
  }

  central_drive_duty = voltageCompensation(central_drive_duty);
  central_drive->setDuty((float)central_drive_duty);
}

void DigController::setBackhoeDuty(double value)
{
  backhoe_duty = clamp(value, -MAX_BACKHOE_DUTY, MAX_BACKHOE_DUTY);
  backhoe_duty = voltageCompensation(backhoe_duty);
  backhoe->setCustom((float)backhoe_duty);
}

void DigController::setBucketDuty(double value)
{
  bucket_duty = clamp(value, -MAX_BUCKET_DUTY, MAX_BUCKET_DUTY);
  bucket_duty = voltageCompensation(bucket_duty);
  bucket->setDuty((float)bucket_duty);
}

void DigController::setVibratorDuty(double value)
{
  vibrator_duty = clamp(value, -MAX_VIBRATOR_DUTY, MAX_VIBRATOR_DUTY);
  vibrator_duty = voltageCompensation(vibrator_duty);
  vibrator->setDuty((float)vibrator_duty);
}

double DigController::getCentralDriveDuty() const
{
  return central_drive_duty;
}

double DigController::getBackhoeDuty() const
{
  return backhoe_duty;
}

double DigController::getBucketDuty() const
{
  return bucket_duty;
}

double DigController::getVibratorDuty() const
{
  return vibrator_duty;
}

double DigController::getCentralDriveCurrent() const
{
  return central_current;
}

double DigController::getBackhoeCurrent() const
{
  return backhoe_current;
}

double DigController::getBucketCurrent() const
{
  return bucket_current;
}

double DigController::getVibratorCurrent() const
{
  return vibrator_current;
}

double DigController::getBatteryVoltage() const
{
  return battery_voltage;
}


int DigController::getCentralDrivePosition() const
{
  return central_drive_position;
}

int DigController::getBackhoePosition() const
{
  return backhoe->getTachometer();
}

double DigController::getBucketPosition() const
{
  return bucket_position;
}

std::string DigController::getCentralDriveStateString() const // Max 20 characters
{
  return to_string(central_drive_state);
}

std::string DigController::getBackhoeStateString() const // Max 9 characters
{
  return to_string(backhoe_state);
}

std::string DigController::getDigStateString() const // Max 15 characters
{
  return to_string(dig_state);
}

std::string DigController::getControlStateString() const
{
  return to_string(goal_state);
}

std::string DigController::getBucketStateString() const
{
  return to_string(bucket_state);
}