#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>

namespace dig_control
{
  class DigController
  {
  public:
    enum class ControlState
    {
      dig,
      finish_dig,
      dump,
      finish_dump,
      ready,
      error,
      manual
    };

    enum class DigState
    {
      dig_transition,
      digging,
      closing_backhoe,
      dump_transition,
      moving_flaps_up,
      stow
    };

    enum class CentralDriveState
    {
      normal,
      at_bottom_limit,
      near_bottom_limit,
      digging,
      near_dump_point,
      at_dump_point,
      flaps_up, // TODO define where this is
      near_top_limit,
      at_top_limit
    };

    enum class BackhoeState
    {
      open,
      closed,
      traveling,
      stuck,
    };

    enum class BucketState
    {
      up,
      down,
      traveling,
      stuck,
    };

    class CentralDriveAngles
    {
    public:
      static constexpr double bottom_limit   = 0.0;
      static constexpr double digging_bottom = 0.1;
      static constexpr double digging_top    = 0.2;
      static constexpr double zero_angle     = 0.3;
      static constexpr double dump_bottom    = 0.4;
      static constexpr double dump_point     = 0.5;
      static constexpr double dump_top       = 0.6;
      static constexpr double near_top_limit = 0.7;
      static constexpr double top_limit      = 0.8;
    };

    class CentralDriveDuty
    {
    public:
      static constexpr float slow = 0.1f;
      static constexpr float normal = 0.2f;
      static constexpr float fast = 0.4f;
    };

    class BackhoeDuty
    {
    public:
      static constexpr float slow = 0.1f;
      static constexpr float normal = 0.4f;
      static constexpr float fast = 0.7f;
    };

    class VibratorDuty
    {
    public:
      static constexpr float normal = 0.7f;
    };

    class BucketDuty
    {
    public:
      static constexpr float fast = 0.4f;
      static constexpr float normal = 0.2f;
    };

    DigController();
    DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator);
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
    float getCentralDriveDuty() const;
    float getBackhoeDuty() const;
    float getBucketDuty() const;
    float getVibratorDuty() const;
    bool isInternallyAllocated();

  private:
    iVescAccess *central_drive, *backhoe, *bucket, *vibrator;
    bool internally_allocated;
    float central_drive_duty, backhoe_duty, bucket_duty, vibrator_duty;

    ControlState goal_state;
    CentralDriveState central_drive_state;
    DigState dig_state;
    BackhoeState backhoe_state;
    BucketState bucket_state;
  };

}

#endif //DIG_CONTROL_2_DIG_CONTROLLER_H
