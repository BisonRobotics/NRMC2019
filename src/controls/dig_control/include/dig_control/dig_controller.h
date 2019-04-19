#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include "../../../../hardware_layer/vesc_access/include/vesc_access/ivesc_access.h"
#include "../../../../hardware_layer/vesc_access/include/vesc_access/vesc_access.h"

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
      at_bottom_limit,
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
      static constexpr int variation      =   10;
      static constexpr int bottom_limit   =  300;
      static constexpr int digging_bottom =  300;
      static constexpr int digging_top    =  900;
      static constexpr int zero_angle     = 1330;
      static constexpr int flaps_bottom   = 2000;
      static constexpr int dump_bottom    = 2250;
      static constexpr int dump_point     = 2300;
      static constexpr int dump_top       = 2350;
      static constexpr int top_limit      = 2550;
    };

    class CentralDriveDuty
    {
    public:
      static constexpr float slow = 0.1f;
      static constexpr float normal = 0.3f;
      static constexpr float fast = 0.5f;
    };

    class BackhoeDuty
    {
    public:
      static constexpr float slow = 0.1f;
      static constexpr float normal = 0.5f;
      static constexpr float fast = 0.8f;
    };

    class VibratorDuty
    {
    public:
      static constexpr float normal = 0.4f;
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
    DigState getDigState() const;
    float getCentralDriveDuty() const;
    float getBackhoeDuty() const;
    float getBucketDuty() const;
    float getVibratorDuty() const;
    int getCentralDrivePosition() const;
    std::string getCentralDriveStateString() const;
    std::string getBackhoeStateString() const;
    std::string getDigStateString() const;
    bool isInternallyAllocated();

  private:
    iVescAccess *central_drive, *backhoe, *bucket, *vibrator;
    bool internally_allocated;
    float central_drive_duty, backhoe_duty, bucket_duty, vibrator_duty;
    int central_drive_position;

    int backhoe_stuck_count;

    ControlState goal_state;
    CentralDriveState central_drive_state;
    DigState dig_state;
    BackhoeState backhoe_state;
    BucketState bucket_state;
  };

}

#endif //DIG_CONTROL_2_DIG_CONTROLLER_H
