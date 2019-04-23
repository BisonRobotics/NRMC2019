#include <competition/controller.h>

using namespace competition;

Controller::Controller(ros::NodeHandle *nh) :
  nh(nh)
{
  joy_subscriber = nh->subscribe("joy", 2, callback)
}
