#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>
#include <dig_control/config.h>

namespace dig_control
{
  class DigController
  {
  public:
    DigController(Config config);
    DigController(Config config, iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator);
    ~DigController();

    void update();
    void updateCentralDriveState();
    void updateBackhoeState();
    void updateBucketState();
    void updateVibratorState();
    void updateBatteryVoltage();

    double voltageCompensation(double duty);
    void setControlState(ControlState goal);
    void setCentralDriveDuty(double value);
    void setBackhoeDuty(double value);
    void setBucketDuty(double value);
    void setVibratorDuty(double value);
    void stop();

    ControlState getControlState() const;
    CentralDriveState getCentralDriveState() const;
    BackhoeState getBackhoeState() const;
    BucketState getBucketState() const;
    DigState getDigState() const;
    double getCentralDriveDuty() const;
    double getBackhoeDuty() const;
    double getBucketDuty() const;
    double getVibratorDuty() const;
    double getCentralDriveCurrent() const;
    double getBackhoeCurrent() const;
    double getBucketCurrent() const;
    double getVibratorCurrent() const;
    double getBatteryVoltage() const;
    int getCentralDrivePosition() const;
    int getBackhoePosition() const;
    double getBucketPosition() const;
    std::string getControlStateString() const;
    std::string getCentralDriveStateString() const;
    std::string getBackhoeStateString() const;
    std::string getDigStateString() const;
    std::string getBucketStateString() const;

    bool isInternallyAllocated();

  private:
    Config config;
    iVescAccess *central_drive, *backhoe, *bucket, *vibrator;
    bool internally_allocated;
    double central_drive_duty, backhoe_duty, bucket_duty, vibrator_duty;
    double backhoe_current, bucket_current, central_current, vibrator_current;
    double battery_voltage;
    int central_drive_position;
    double bucket_position;

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
