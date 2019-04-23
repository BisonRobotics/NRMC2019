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

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  this->joy = joy;
}
