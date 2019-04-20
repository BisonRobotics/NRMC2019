#ifndef DIG_CONTROL_2_DIG_CONTROLLER_H
#define DIG_CONTROL_2_DIG_CONTROLLER_H

#include <dig_control/dig_controller_interface.h>
#include <vesc_access/ivesc_access.h>
#include <vesc_access/vesc_access.h>

namespace dig_control
{

  class DigController : public DigControllerInterface
  {
  public:
    DigController();
    DigController(iVescAccess *central_drive,   iVescAccess *backhoe_actuator,
                  iVescAccess *bucket_actuator, iVescAccess *vibrator);
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
    int getCentralDrivePosition() const override;
    std::string getControlStateString() const override;
    std::string getCentralDriveStateString() const override;
    std::string getBackhoeStateString() const override;
    std::string getDigStateString() const override;
    std::string getBucketStateString() const override;

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
