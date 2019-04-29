#include <dig_control/dig_control_client.h>

using namespace dig_control;

DigControlClient::DigControlClient() :
  client("dig_control_server/action", true),
  boostDoneCallback(boost::bind(&DigControlClient::doneCallback, this, _1, _2)),
  boostActiveCallback(boost::bind(&DigControlClient::activeCallback, this)),
  boostFeedbackCallback(boost::bind(&DigControlClient::feedbackCallback, this, _1)),
  control_state(ControlState::ready),
  dig_state(DigState::dig_transition), central_drive_state(CentralDriveState::near_dump_point),
  backhoe_state(BackhoeState::open), bucket_state(BucketState::down)
{
  ROS_INFO("Waiting for dig_control_server to start");
  client.waitForServer();
  ROS_INFO("Connected to dig_control_server");
}

void DigControlClient::setControlState(ControlState state)
{
  dig_control::DigControlGoal goal;
  goal.control_state = (uint8_t)state;
  control_state = state;
  client.sendGoal(goal, boostDoneCallback, boostActiveCallback, boostFeedbackCallback);
}

void DigControlClient::doneCallback(const actionlib::SimpleClientGoalState &state,
                                    const dig_control::DigControlResultConstPtr &result)
{
  control_state = (ControlState)result->control_state;
}

void DigControlClient::activeCallback()
{
  ROS_INFO("[DigControlClient::activeCallback]");
}

void DigControlClient::feedbackCallback(const dig_control::DigControlFeedbackConstPtr &feedback)
{
  control_state       =      (ControlState)feedback->control_state;
  dig_state           =          (DigState)feedback->dig_state;
  central_drive_state = (CentralDriveState)feedback->central_drive_state;
  backhoe_state       =      (BackhoeState)feedback->backhoe_state;
  bucket_state        =       (BucketState)feedback->bucket_state;
  /*ROS_INFO("| CS | %s | DS | %s | CD | %s | BH | %s |",
    to_string(control_state).c_str(),
    to_string(dig_state).c_str(),
    to_string(central_drive_state).c_str(),
    to_string(backhoe_state).c_str());*/
}

ControlState DigControlClient::getControlState() const
{
  return control_state;
}

DigState DigControlClient::getDigState() const
{
  return dig_state;
}

CentralDriveState DigControlClient::getCentralDriveState() const
{
  return central_drive_state;
}

BackhoeState DigControlClient::getBackhoeState() const
{
  return backhoe_state;
}

BucketState DigControlClient::getBucketState() const
{
  return bucket_state;
}
