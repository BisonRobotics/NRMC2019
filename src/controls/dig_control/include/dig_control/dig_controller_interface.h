#ifndef DIG_CONTROL_DIG_CONTROLLER_INTERFACE_H
#define DIG_CONTROL_DIG_CONTROLLER_INTERFACE_H

#include <string>

namespace dig_control
{
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

  class CentralDriveAngles
  {
  public:
    static constexpr int variation      =   10;
    static constexpr int bottom_limit   =  300;
    static constexpr int digging_bottom =  300;
    static constexpr int digging_top    =  900;
    static constexpr int floor_limit    = 1250;//1146;
    static constexpr int zero_angle     = 1330;
    static constexpr int stow_position  = 1650;
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
    //static constexpr float normal = 0.4f;
    static constexpr float normal = 0.0f;
  };

  class BucketDuty
  {
  public:
    static constexpr float fast = 0.4f;
    static constexpr float normal = 0.2f;
  };


  class DigControllerInterface
  {
  public:

    virtual void update() = 0;

    virtual void setControlState(ControlState goal) = 0;
    virtual void setCentralDriveDuty(float value) = 0;
    virtual void setBackhoeDuty(float value) = 0;
    virtual void setBucketDuty(float value) = 0;
    virtual void setVibratorDuty(float value) = 0;
    virtual void stop() = 0;

    virtual ControlState getControlState() const = 0;
    virtual CentralDriveState getCentralDriveState() const = 0;
    virtual BackhoeState getBackhoeState() const = 0;
    virtual BucketState getBucketState() const = 0;
    virtual DigState getDigState() const = 0;
    virtual float getCentralDriveDuty() const = 0;
    virtual float getBackhoeDuty() const = 0;
    virtual float getBucketDuty() const = 0;
    virtual float getVibratorDuty() const = 0;
    virtual float getCentralDriveCurrent() const = 0;
    virtual float getBackhoeCurrent() const = 0;
    virtual float getBucketCurrent() const = 0;
    virtual float getVibratorCurrent() const = 0;
    virtual int getCentralDrivePosition() const = 0;
    virtual int getBackhoePosition() const = 0;
    virtual std::string getControlStateString() const = 0;
    virtual std::string getCentralDriveStateString() const = 0;
    virtual std::string getBackhoeStateString() const = 0;
    virtual std::string getDigStateString() const = 0;
    virtual std::string getBucketStateString() const = 0;
  };

}

#endif //DIG_CONTROL_DIG_CONTROLLER_INTERFACE_H