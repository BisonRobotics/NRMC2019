#include <dig_control/config.h>

using namespace dig_control;

Config::Config(ros::NodeHandle *nh) : utilities::Config("dig_control")
{
  loadParam(nh, "floor_test", floor_test, true);
}

const CentralDriveAngles &Config::centralDriveAngles()
{
  return central_drive_angles;
}

const CentralDriveDuty &Config::centralDriveDuty()
{
  return central_drive_duty;
}

const BackhoeDuty &Config::backhoeDuty()
{
  return backhoe_duty;
}

const VibratorDuty &Config::vibratorDuty()
{
  return vibrator_duty;
}

const BucketDuty &Config::bucketDuty()
{
  return bucket_duty;
}

bool Config::floorTest()
{
  return floor_test;
}
