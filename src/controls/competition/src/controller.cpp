#include <competition/controller.h>

using namespace competition;

Controller::Controller(ros::NodeHandle *nh, ros::Rate *rate) :
  nh(nh), rate(rate), dt(rate->expectedCycleTime().toSec())
{
  joy_subscriber = nh->subscribe("joy", 1, &Controller::joyCallback, this);
}

void Controller::update()
{

}

void Controller::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  this->joy = joy;
}
