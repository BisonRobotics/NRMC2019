#include <competition/controller.h>

using namespace competition;

Controller::Controller(ros::NodeHandle *nh, ros::Rate *rate) :
  nh(nh), rate(rate), dt(rate->expectedCycleTime().toSec()),
  visuals(nh)
{
  joy_subscriber = nh->subscribe("joy", 1, &Controller::joyCallback, this);
  drive_client.setControlState(DriveControlState::manual);
  dig_client.setControlState(DigControlState::manual);
}

void Controller::update()
{
  visuals.update();
}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
  this->joy = joy_msg;
  
  if (joy.get(Joy::AUTONOMY_SAFETY))
  {
    if (joy.get(Joy::START_DIG)) 
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
          to_string(dig_client.getControlState()).c_str(),
          to_string(DigControlState::dig).c_str());
      dig_client.setControlState(DigControlState::dig);
    }
    else if (joy.get(Joy::END_DIG))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::finish_dig).c_str());
      dig_client.setControlState(DigControlState::finish_dig);
    }
    else if (joy.get(Joy::START_DUMP))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::dump).c_str());
      dig_client.setControlState(DigControlState::dump);
    }
    else if (joy.get(Joy::END_DUMP))
    {
      ROS_INFO("[Controller::joyCallback]: %s to %s",
               to_string(dig_client.getControlState()).c_str(),
               to_string(DigControlState::finish_dump).c_str());
      dig_client.setControlState(DigControlState::finish_dump);
    }
  }
  else
  {
    if (dig_client.getControlState() != DigControlState::manual)
    {
      dig_client.setControlState(DigControlState::manual);
    }
  }
}
