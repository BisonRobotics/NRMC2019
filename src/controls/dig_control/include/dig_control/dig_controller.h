#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>
#include <dig_control/config.h>

namespace dig_control
{
  /**
 * A simple low pass filter from the VESC firmware
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
  template <typename type>
  void lowPassFilter(type &value, type sample, type filter_constant)
  {
    value -= filter_constant * (value - sample);
  }

  enum class ControlState
  {
    error = 0,
    ready,
    manual,
    dig,
    finish_dig,
    dump,
    finish_dump
  };

  enum class DigState
  {
    dig_transition = 0,
    digging,
    closing_backhoe,
    dump_transition,
    moving_flaps_up,
    stow
  };

  enum class CentralDriveState
  {
    at_bottom_limit = 0,
    digging,
    near_digging,
    flap_transition_down,
    near_dump_point,
    at_dump_point,
    flap_transition_up,
    at_top_limit
  };

  enum class BackhoeState
  {
    open = 0,
    closed,
    traveling,
    stuck,
  };

  enum class BucketState
  {
    up = 0,
    down,
    traveling,
    stuck,
  };

  std::string to_string(ControlState state);
  std::string to_string(DigState state);
  std::string to_string(CentralDriveState state);
  std::string to_string(BackhoeState state);
  std::string to_string(BucketState state);
  
  class DigController
  {
  public:
    DigController(Config *config);
    DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator, Config *config);
    ~DigController();

    void update();
    void updateCentralDriveState();
    void updateBackhoeState();
    void updateBucketState();

    void setControlState(ControlState goal);
    void setCentralDriveDuty(float value);
    void setBackhoeDuty(float value);
    void setBucketDuty(float value);
    void setVibratorDuty(float value);
    void stop();

    ControlState getControlState() const;
    CentralDriveState getCentralDriveState() const;
    BackhoeState getBackhoeState() const;
    BucketState getBucketState() const;
    DigState getDigState() const;
    float getCentralDriveDuty() const;
    float getBackhoeDuty() const;
    float getBucketDuty() const;
    float getVibratorDuty() const;
    float getCentralDriveCurrent() const;
    float getBackhoeCurrent() const;
    float getBucketCurrent() const;
    float getVibratorCurrent() const;
    int getCentralDrivePosition() const;
    int getBackhoePosition() const;
    float getBucketPosition() const;
    std::string getControlStateString() const;
    std::string getCentralDriveStateString() const;
    std::string getBackhoeStateString() const;
    std::string getDigStateString() const;
    std::string getBucketStateString() const;

    bool isInternallyAllocated();

  private:
    Config *config;
    iVescAccess *central_drive, *backhoe, *bucket, *vibrator;
    bool internally_allocated;
    bool floor_test;
    float central_drive_duty, backhoe_duty, bucket_duty, vibrator_duty;
    float backhoe_current, bucket_current, central_current, vibrator_current;
    int central_drive_position;
    float bucket_position;

    int backhoe_stuck_count;

    ros::Time last_bucket_state_change;

    ControlState goal_state;
    CentralDriveState central_drive_state;
    DigState dig_state;
    BackhoeState backhoe_state;
    BucketState bucket_state;
  };

}

#endif //DIG_CONTROL_2_DIG_CONTROLLER_H
