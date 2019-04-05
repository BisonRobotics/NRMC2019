#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>

namespace dig_control_2
{
  class DigController
  {
  public:
    enum class Goal
    {
      drive_pose,
      dig,
      dump,
      stop,
      manual
    };

    enum class Status
    {
      ready,
      in_progress,
      error
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

    /*
          switch (central_drive_zone)
          {
            case CentralDriveZone::normal:
            case CentralDriveZone::at_bottom_limit:
            case CentralDriveZone::near_bottom_limit:
            case CentralDriveZone::digging:
            case CentralDriveZone::near_dump_point:
            case CentralDriveZone::at_dump_point:
            case CentralDriveZone::flaps_up:
            case CentralDriveZone::near_top_limit:
            case CentralDriveZone::at_top_limit:
          }

          switch (backhoe_state)
          {
            case BackhoeState::closed:
            case BackhoeState::traveling:
            case BackhoeState::open:
            case BackhoeState::stuck:
          }
     */

    enum class DigState
    {
      stowed,
      dig_transition,
      digging,
      closing_backhoe,
      dump_transition,
      dumping,
      moving_flaps_up,
      error,
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
      static constexpr double bottom_limit = 0.0;
      static constexpr double digging_bottom = 0.0;
      static constexpr double digging_top = 0.0;
      static constexpr double zero_angle = 0.0;
      static constexpr double dump_bottom = 0.0;
      static constexpr double dump_point = 0.0;
      static constexpr double dump_top = 0.0;
      static constexpr double near_top_limit = 0.0;
      static constexpr double top_limit = 0.0;
    };

    class CentralDriveDuty
    {
    public:
      static constexpr float slow = 0.1f;
      static constexpr float normal = 0.4f;
    };

    class BackhoeDuty
    {
    public:
      static constexpr float slow = 0.1f;
      static constexpr float normal = 0.4f;
    };

    class VibratorDuty
    {
    public:
      static constexpr float normal = 0.4f;
    };

    DigController();
    DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator);
    ~DigController();

    bool isInternallyAllocated();

    void update();
    void updateCentralDriveState();
    void updateBackhoeState();
    void updateBucketState();

    void setGoal(Goal goal);
    void stop();
    void setCentralDriveDuty(float value);
    void setBackhoeDuty(float value);
    void setBucketDuty(float value);
    void setVibratorDuty(float value);

    Goal getGoal() const;
    Status getStatus() const;
    double getProgress() const;
    CentralDriveState getCentralDriveState() const;
    BackhoeState getBackhoeState() const;
    BucketState getBucketState() const;
    float getCentralDriveDuty() const;
    float getBackhoeDuty() const;
    float getBucketDuty() const;
    float getVibratorDuty() const;


  private:
    iVescAccess *central_drive, *backhoe, *bucket, *vibrator;
    bool internally_allocated;
    float central_drive_duty, backhoe_duty, bucket_duty, vibrator_duty;

    Goal goal;
    Status status;
    double progress;
    CentralDriveState central_drive_state;
    DigState dig_state;
    BackhoeState backhoe_state;
    BucketState bucket_state;
  };

}

#endif //DIG_CONTROL_2_DIG_CONTROLLER_H
