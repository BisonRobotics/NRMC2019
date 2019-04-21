#ifndef DIG_CONTROL_DIG_CONTROLLER_SIM_H
#define DIG_CONTROL_DIG_CONTROLLER_SIM_H

#include <dig_control/dig_controller_interface.h>
#include <vector>

namespace dig_control
{

  class DigControllerSim : public DigControllerInterface
  {
  public:
    DigControllerSim(std::string file);

    void update() override;

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
    int getBackhoePosition() const override;
    float getBucketPosition() const override;
    std::string getControlStateString() const override;
    std::string getCentralDriveStateString() const override;
    std::string getBackhoeStateString() const override;
    std::string getDigStateString() const override;
    std::string getBucketStateString() const override;

  private:
    uint i;
    bool increment;
    int size;
    ControlState control_state;
    std::vector<float> backhoe_duty, bucket_duty, central_duty, vibrator_duty;
    std::vector<ControlState> goal_state;
    std::vector<CentralDriveState> central_drive_state;
    std::vector<DigState> dig_state;
    std::vector<BackhoeState> backhoe_state;
    std::vector<BucketState> bucket_state;
    std::vector<int> central_position;
  };

}

#endif //DIG_CONTROL_DIG_CONTROLLER_SIM_H
