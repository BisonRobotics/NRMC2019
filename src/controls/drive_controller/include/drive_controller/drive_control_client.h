#ifndef DRIVE_CONTROLLER_DRIVE_CONTROL_CLIENT_H
#define DRIVE_CONTROLLER_DRIVE_CONTROL_CLIENT_H

#include <drive_controller/drive_control_server.h>
#include <drive_controller/DriveControlAction.h>
#include <navigation_msgs/BezierSegment.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_goal_state.h>

namespace drive_controller
{
  class DriveControlClient
  {
  public:
    DriveControlClient();

    void setControlState(ControlState state);
    void setControlState(ControlState state, navigation_msgs::BezierSegment segment);

    void doneCallback(const actionlib::SimpleClientGoalState &state, const DriveControlResultConstPtr &result);
    void activeCallback();
    void feedbackCallback(const DriveControlFeedbackConstPtr &feedback);

    ControlState getControlState() const;
    double getProgress() const;
    double getDeviation() const;

  private:
    actionlib::SimpleActionClient<drive_controller::DriveControlAction> client;
    boost::function<void (const actionlib::SimpleClientGoalState&,
                          const DriveControlResultConstPtr&)> boostDoneCallback;
    boost::function<void ()> boostActiveCallback;
    boost::function<void (const DriveControlFeedbackConstPtr&)> boostFeedbackCallback;

    ControlState control_state;
    double progress, deviation;
  };
}

#endif //DRIVE_CONTROLLER_DRIVE_CONTROL_CLIENT_H
