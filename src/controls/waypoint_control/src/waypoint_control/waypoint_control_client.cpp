#include <waypoint_control/waypoint_control_client.h>

using namespace waypoint_control;

WaypointControlClient::WaypointControlClient() :
  client("waypoint_control_server/action", true),
  boostDoneCallback(boost::bind(&WaypointControlClient::doneCallback, this, _1, _2)),
  boostActiveCallback(boost::bind(&WaypointControlClient::activeCallback, this)),
  boostFeedbackCallback(boost::bind(&WaypointControlClient::feedbackCallback, this, _1)),
  control_state(ControlState::manual)
{
  ROS_INFO("Waiting for waypoint_control_server to start");
  client.waitForServer();
  ROS_INFO("Connected to waypoint_control_server");
}

void WaypointControlClient::setControlState(ControlState state)
{
  control_state = state;
  waypoint_control::WaypointControlGoal goal;
  goal.control_state = (uint8_t)state;
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void WaypointControlClient::setControlState(ControlState state, const Waypoints &waypoints)
{
  control_state = state;
  waypoint_control::WaypointControlGoal goal;
  goal.control_state = (uint8_t)state;
  goal.waypoints = waypoints;
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void WaypointControlClient::doneCallback(const actionlib::SimpleClientGoalState &state,
                                      const WaypointControlResultConstPtr &result)
{
  ROS_INFO("[WaypointControlClient::doneCallback]: %s to %s",
      to_string(control_state).c_str(), to_string((ControlState)result->control_state).c_str());
  control_state = (ControlState)result->control_state;
  progress = result->progress;
  angular_deviation = result->angular_deviation;
  linear_deviation = result->linear_deviation;
}

void WaypointControlClient::activeCallback()
{
  ROS_INFO("[WaypointControlClient::activeCallback]");
}

void WaypointControlClient::feedbackCallback(const WaypointControlFeedbackConstPtr &feedback)
{
  control_state = (ControlState)feedback->control_state;
  progress = feedback->progress;
  angular_deviation = feedback->angular_deviation;
  linear_deviation = feedback->linear_deviation;
}

ControlState WaypointControlClient::getControlState() const
{
  return control_state;
}

double WaypointControlClient::getProgress() const
{
  return progress;
}

double WaypointControlClient::getLinearDeviation() const
{
  return linear_deviation;
}

double WaypointControlClient::getAngularDeviation() const
{
  return angular_deviation;
}
