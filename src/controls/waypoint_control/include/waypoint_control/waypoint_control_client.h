#ifndef WAYPOINT_CONTROL_WAYPOINT_CONTROL_CLIENT_H
#define WAYPOINT_CONTROL_WAYPOINT_CONTROL_CLIENT_H

#include <waypoint_control/waypoint_control_server.h>
#include <waypoint_control/WaypointControlAction.h>
#include <waypoint_control/Waypoint.h>
#include <navigation_msgs/BezierSegment.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_goal_state.h>
#include <vector>

namespace waypoint_control
{
  class WaypointControlClient
  {
  public:
    WaypointControlClient();

    void setControlState(ControlState state);
    void setControlState(ControlState state, const Waypoints &waypoints);

    void doneCallback(const actionlib::SimpleClientGoalState &state, const WaypointControlResultConstPtr &result);
    void activeCallback();
    void feedbackCallback(const WaypointControlFeedbackConstPtr &feedback);

    ControlState getControlState() const;
    double getProgress() const;
    double getLinearDeviation() const;
    double getAngularDeviation() const;

  private:
    actionlib::SimpleActionClient<waypoint_control::WaypointControlAction> client;
    boost::function<void (const actionlib::SimpleClientGoalState&,
                          const WaypointControlResultConstPtr&)> boostDoneCallback;
    boost::function<void ()> boostActiveCallback;
    boost::function<void (const WaypointControlFeedbackConstPtr&)> boostFeedbackCallback;

    ControlState control_state;
    double progress, linear_deviation, angular_deviation;
  };
}

#endif //WAYPOINT_CONTROL_WAYPOINT_CONTROL_CLIENT_H
