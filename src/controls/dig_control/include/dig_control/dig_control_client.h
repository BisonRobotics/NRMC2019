#ifndef DIG_CONTROL_DIG_CONTROL_CLIENT_H
#define DIG_CONTROL_DIG_CONTROL_CLIENT_H

#include <ros/ros.h>
#include <dig_control/DigControlAction.h>
#include <dig_control/dig_controller.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"

namespace dig_control
{
  class DigControlClient
  {
  public:
    DigControlClient();

    void setControlState(ControlState state);

    void doneCallback(const actionlib::SimpleClientGoalState &state, const DigControlResultConstPtr &result);
    void activeCallback();
    void feedbackCallback(const DigControlFeedbackConstPtr &feedback);

    ControlState getControlState() const;
    DigState getDigState() const;
    CentralDriveState getCentralDriveState() const;
    BackhoeState getBackhoeState() const;
    BucketState getBucketState() const;

  private:
    actionlib::SimpleActionClient<dig_control::DigControlAction> client;
    boost::function<void (const actionlib::SimpleClientGoalState&, const DigControlResultConstPtr&)> boostDoneCallback;
    boost::function<void ()> boostActiveCallback;
    boost::function<void (const DigControlFeedbackConstPtr&)> boostFeedbackCallback;

    ControlState control_state;
    DigState dig_state;
    CentralDriveState central_drive_state;
    BackhoeState backhoe_state;
    BucketState bucket_state;
  };
}

#endif //DIG_CONTROL_DIG_CONTROL_CLIENT_H
