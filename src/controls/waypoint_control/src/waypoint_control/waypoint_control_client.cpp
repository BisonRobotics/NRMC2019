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
  waypoint_control::WaypointControlGoal goal;
  goal.control_state = (uint8_t)state;
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void WaypointControlClient::setControlState(ControlState state, navigation_msgs::BezierSegment segment)
{
  waypoint_control::WaypointControlGoal goal;
  goal.control_state = (uint8_t)state;
  goal.path.emplace_back(segment);
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void WaypointControlClient::doneCallback(const actionlib::SimpleClientGoalState &state,
                                      const WaypointControlResultConstPtr &result)
{
  control_state = (ControlState)result->control_state;
  progress = result->progress;
  deviation = result->deviation;
}

void WaypointControlClient::activeCallback()
{
  ROS_INFO("[WaypointControlClient::activeCallback]");
}

void WaypointControlClient::feedbackCallback(const WaypointControlFeedbackConstPtr &feedback)
{
  control_state = (ControlState)feedback->control_state;
  progress = feedback->progress;
  deviation = feedback->deviation;
}

ControlState WaypointControlClient::getControlState() const
{
  return control_state;
}

double WaypointControlClient::getProgress() const
{
  return progress;
}

double WaypointControlClient::getDeviation() const
{
  return deviation;
}
