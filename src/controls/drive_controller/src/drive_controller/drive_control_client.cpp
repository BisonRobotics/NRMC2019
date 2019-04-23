#include <drive_controller/drive_control_client.h>

using namespace drive_controller;

DriveControlClient::DriveControlClient() :
  client("drive_control_server/action", true),
  boostDoneCallback(boost::bind(&DriveControlClient::doneCallback, this, _1, _2)),
  boostActiveCallback(boost::bind(&DriveControlClient::activeCallback, this)),
  boostFeedbackCallback(boost::bind(&DriveControlClient::feedbackCallback, this, _1)),
  control_state(ControlState::ready)
{
  ROS_INFO("Waiting for drive_control_server to start");
  client.waitForServer();
  ROS_INFO("Connected to drive_control_server");
}

void DriveControlClient::setControlState(ControlState state)
{
  drive_controller::DriveControlGoal goal;
  goal.control_state = (uint8_t)state;
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void DriveControlClient::setControlState(ControlState state, navigation_msgs::BezierSegment segment)
{
  drive_controller::DriveControlGoal goal;
  goal.control_state = (uint8_t)state;
  goal.path.emplace_back(segment);
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void DriveControlClient::doneCallback(const actionlib::SimpleClientGoalState &state,
                                      const DriveControlResultConstPtr &result)
{
  control_state = (ControlState)result->control_state;
  progress = result->progress;
  deviation = result->deviation;
}

void DriveControlClient::activeCallback()
{
  ROS_INFO("[DriveControlClient::activeCallback]");
}

void DriveControlClient::feedbackCallback(const DriveControlFeedbackConstPtr &feedback)
{
  control_state = (ControlState)feedback->control_state;
  progress = feedback->progress;
  deviation = feedback->deviation;
}

ControlState DriveControlClient::getControlState() const
{
  return control_state;
}

double DriveControlClient::getProgress() const
{
  return progress;
}

double DriveControlClient::getDeviation() const
{
  return deviation;
}
