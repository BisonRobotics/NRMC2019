#ifndef DIG_CONTROL_CONFIG_H
#define DIG_CONTROL_CONFIG_H

#include <utilities/config.h>

namespace dig_control
{
  class CentralDriveAngles
  {
  public:
    const int variation      =   10;
    const int bottom_limit   =  300;
    const int digging_bottom =  300;
    const int digging_top    =  900;
    const int floor_limit    = 1250;//1146;
    const int zero_angle     = 1330;
    const int stow_position  = 1650;
    const int flaps_bottom   = 2000;
    const int dump_bottom    = 2250;
    const int dump_point     = 2300;
    const int dump_top       = 2350;
    const int top_limit      = 2550;
  };

  class CentralDriveDuty
  {
  public:
    const float slow = 0.1f;
    const float normal = 0.3f;
    const float fast = 0.5f;
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
    //static constexpr float normal = 0.4f;
    const float normal = 0.0f;
  };

  class BucketDuty
  {
  public:
    const float fast = 0.4f;
    const float normal = 0.2f;
  };

  class Config : public utilities::Config
  {
  public:
    Config(ros::NodeHandle *nh);

    bool floorTest();
    const CentralDriveAngles& centralDriveAngles();
    const CentralDriveDuty& centralDriveDuty();
    const BackhoeDuty& backhoeDuty();
    const VibratorDuty& vibratorDuty();
    const BucketDuty& bucketDuty();

  private:
    bool floor_test;
    CentralDriveAngles central_drive_angles;
    CentralDriveDuty central_drive_duty;
    BackhoeDuty backhoe_duty;
    VibratorDuty vibrator_duty;
    BucketDuty bucket_duty;
  };
}

#endif //DIG_CONTROL_CONFIG_H
