#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <dig_control/dig_controller_interface.h>
#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>
#include <string>

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

  class DigController : public DigControllerInterface
  {
  public:
    DigController(bool floor_test = false);
    DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator, bool floor_test = false);
    ~DigController();

    void update() override;
    void updateCentralDriveState();
    void updateBackhoeState();
    void updateBucketState();

    void setControlState(ControlState goal) override;
    void setCentralDriveDuty(float value) override;
    void setBackhoeDuty(float value) override;
    void setBucketDuty(float value) override;
    void setVibratorDuty(float value) override;
    void stop();

    ControlState getControlState() const override;
    CentralDriveState getCentralDriveState() const override;
    BackhoeState getBackhoeState() const override;
    BucketState getBucketState() const override;
    DigState getDigState() const override;
    float getCentralDriveDuty() const override;
    float getBackhoeDuty() const override;
    float getBucketDuty() const override;
    float getVibratorDuty() const override;
    float getCentralDriveCurrent() const override;
    float getBackhoeCurrent() const override;
    float getBucketCurrent() const override;
    float getVibratorCurrent() const override;
    int getCentralDrivePosition() const override;
    int getBackhoePosition() const override;
    float getBucketPosition() const override;
    std::string getControlStateString() const override;
    std::string getCentralDriveStateString() const override;
    std::string getBackhoeStateString() const override;
    std::string getDigStateString() const override;
    std::string getBucketStateString() const override;

    bool isInternallyAllocated();

  private:
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
