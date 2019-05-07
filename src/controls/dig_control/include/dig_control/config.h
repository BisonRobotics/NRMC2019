#ifndef DIG_CONTROL_CONFIG_H
#define DIG_CONTROL_CONFIG_H

#include <utilities/config.h>

namespace dig_control
{
  class CentralDriveAngles
  {
  public:
    const int variation      =   10;
    const int bottom_limit   =  150; //Test pit 500; // Arena 150;
    const int digging_bottom =  150;
    const int digging_top    =  900;
    const int floor_limit    = 1250; //1146;
    const int zero_angle     = 1330;
    const int stow_position  = 1650;
    const int flaps_bottom   = 2000;
    const int dump_bottom    = 2250;
    const int dump_point     = 2300;
    const int dump_top       = 2350;
    const int top_limit      = 2500; //2550;
  };

  class CentralDriveDuty
  {
  public:
    const float ultra_slow = 0.05f;
    const float slow = 0.1f;
    const float slowish = 0.18f;
    const float normal = 0.3f;
    const float fast = 0.5f;
    const float max = 0.5f;
  };

  class BackhoeDuty
  {
  public:
    const float slow = 0.1f;
    const float normal = 0.5f;
    const float fast = 0.8f;
  };

  class VibratorDuty
  {
  public:
    const float normal = 0.8f;
    //const float normal = 0.0f;
  };

  class BucketDuty
  {
  public:
    const float fast = 0.4f;
    const float normal = 0.2f;
  };

  enum class ControlState
  {
    error = 0,
    ready,
    manual,
    initialize,
    dig,
    finish_dig,
    dump,
    finish_dump
  };

  enum class DigState
  {
    initialize = 0,
    dig_transition,
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

  class Config : public utilities::Config
  {
  public:
    explicit Config(ros::NodeHandle *nh);

    const CentralDriveAngles& centralDriveAngles();
    const CentralDriveDuty& centralDriveDuty();
    const BackhoeDuty& backhoeDuty();
    const VibratorDuty& vibratorDuty();
    const BucketDuty& bucketDuty();
    const bool &debug();
    const bool &floorTest();
    const bool &fullAutonomy();
    const bool   &voltageCompensation();
    const double &minVoltage();
    const double &startVoltage();
    const double &maxCompensatedDuty();
    const double &batteryFilterK();
    const double &centralDriveAngleFilterK();
    const double &centralDriveCurrentFilterK();
    const double &centralDriveDigCurrentThreshold();
    const double &currentFilterK();
    const double &bucketFilterK();

  private:
    bool debug_;
    bool floor_test_;
    bool full_autonomy_;
    CentralDriveAngles central_drive_angles_;
    CentralDriveDuty central_drive_duty_;
    BackhoeDuty backhoe_duty_;
    VibratorDuty vibrator_duty_;
    BucketDuty bucket_duty_;
    bool voltage_compensation_;
    double min_voltage_;
    double full_voltage_;
    double max_compensated_duty_;
    double battery_filter_k_;
    double central_drive_angle_filter_k_;
    double current_filter_k_;
    double central_drive_dig_current_threshold_;
    double central_drive_current_filter_k_;
    double bucket_filter_k_;
  };
}

#endif //DIG_CONTROL_CONFIG_H
